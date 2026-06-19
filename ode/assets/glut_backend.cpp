/*************************************************************************
 * glut_backend.cpp                                                       *
 *                                                                        *
 * A native macOS windowing backend for ODE's DrawStuff, built on the     *
 * system GLUT.framework (Cocoa-backed, no XQuartz required).             *
 *                                                                        *
 * Why this file exists:                                                   *
 *   DrawStuff is split into a platform-INDEPENDENT core (drawstuff.cpp,  *
 *   all the dsDrawBox/lighting/texture immediate-mode GL) and a thin     *
 *   platform backend that owns the window + GL context. The stock macOS  *
 *   backend (drawstuff/src/osx.cpp) uses 32-bit Carbon + AGL, which was  *
 *   removed from modern macOS and does not build on arm64. This file is  *
 *   a drop-in replacement for that backend.                             *
 *                                                                        *
 * It implements precisely the contract declared in drawstuff/src/        *
 * internal.h:                                                            *
 *   - supplies:  dsPlatformSimLoop, dsError, dsDebug, dsPrint, dsStop,   *
 *                dsElapsedTime                                            *
 *   - and calls into the core:  dsStartGraphics, dsDrawFrame,           *
 *                dsStopGraphics, dsMotion, dsGet/SetTextures,            *
 *                dsGet/SetShadows, dsGet/SetViewpoint                    *
 *                                                                        *
 * The core's dsDrawFrame() is what invokes the user's fn->step(pause)    *
 * callback each frame (drawstuff.cpp:1176), so this backend only has to  *
 * pump events, call dsDrawFrame, and swap buffers.                       *
 *                                                                        *
 * License: this backend is released under the same dual BSD/LGPL terms   *
 * as the rest of DrawStuff (see third_party/drawstuff/LICENSE-BSD.TXT).  *
 *************************************************************************/

#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cstring>
#include <cctype>
#include <sys/time.h>

/* Silence the (expected) "OpenGL is deprecated" warnings: DrawStuff is a
 * legacy immediate-mode renderer and that is exactly what we want here. */
#ifndef GL_SILENCE_DEPRECATION
#define GL_SILENCE_DEPRECATION 1
#endif
#include <GLUT/glut.h>
#include <OpenGL/gl.h>

#include <drawstuff/drawstuff.h>
#include <drawstuff/version.h>
#include "internal.h"

/* ----------------------------------------------------------------------- */
/* backend state                                                           */
/* ----------------------------------------------------------------------- */

static dsFunctions *g_fn        = 0;
static int          g_width     = 0;
static int          g_height    = 0;
static int          g_pausemode = 0;   /* 1 == paused                       */
static int          g_singlestep= 0;   /* advance one frame while paused    */
static int          g_mouse_mode= 0;   /* bit0 left, bit1 middle, bit2 right*/
static int          g_mouse_x   = 0;
static int          g_mouse_y   = 0;

/* ----------------------------------------------------------------------- */
/* error / logging helpers (the extern "C" trio drawstuff.h declares)      */
/* ----------------------------------------------------------------------- */

static void printMessage(const char *kind, const char *msg, va_list ap)
{
  fflush(stderr); fflush(stdout);
  fprintf(stderr, "\n%s: ", kind);
  vfprintf(stderr, msg, ap);
  fprintf(stderr, "\n");
  fflush(stderr);
}

extern "C" void dsError(const char *msg, ...)
{
  va_list ap; va_start(ap, msg);
  printMessage("Error", msg, ap);
  va_end(ap);
  exit(1);
}

extern "C" void dsDebug(const char *msg, ...)
{
  va_list ap; va_start(ap, msg);
  printMessage("INTERNAL ERROR", msg, ap);
  va_end(ap);
  abort();
}

extern "C" void dsPrint(const char *msg, ...)
{
  va_list ap; va_start(ap, msg);
  vprintf(msg, ap);
  va_end(ap);
}

/* seconds elapsed since the previous call (clamped, like the X11 backend) */
extern "C" double dsElapsedTime(void)
{
  static double prev = 0.0;
  struct timeval tv;
  gettimeofday(&tv, 0);
  double curr = tv.tv_sec + (double)tv.tv_usec / 1000000.0;
  if (prev == 0.0) prev = curr;
  double dt = curr - prev;
  prev = curr;
  if (dt > 1.0) dt = 1.0;
  if (dt < 1.0e-7) dt = 1.0e-7;
  return dt;
}

/* ----------------------------------------------------------------------- */
/* graceful shutdown                                                       */
/* ----------------------------------------------------------------------- */
/* Apple's GLUT glutMainLoop() does not return, so we cannot "return to the
 * caller" the way the X11 backend does. Instead we shut the renderer down
 * cleanly and exit(); the demo registers its ODE teardown (dCloseODE, ...)
 * with atexit(), so the full ODE lifecycle still runs on the way out. */
static void shutdownAndExit(void)
{
  static int done = 0;
  if (done) return;
  done = 1;
  if (g_fn && g_fn->stop) g_fn->stop();
  dsStopGraphics();
  exit(0);
}

extern "C" void dsStop(void)
{
  shutdownAndExit();
}

/* ----------------------------------------------------------------------- */
/* GLUT callbacks                                                          */
/* ----------------------------------------------------------------------- */

static void displayCB(void)
{
  /* The core does all camera/projection/lighting setup and calls
   * g_fn->step(pause) internally. */
  dsDrawFrame(g_width, g_height, g_fn, g_pausemode && !g_singlestep);
  g_singlestep = 0;
  glutSwapBuffers();
}

static void reshapeCB(int w, int h)
{
  g_width  = w;
  g_height = h;
  glViewport(0, 0, w, h);   /* dsDrawFrame also sets this; harmless */
}

/* ~60 Hz animation tick */
static void timerCB(int /*value*/)
{
  glutPostRedisplay();
  glutTimerFunc(16, timerCB, 0);
}

static void keyboardCB(unsigned char key, int /*x*/, int /*y*/)
{
  if (key == 27) {                 /* ESC -> quit */
    shutdownAndExit();
    return;
  }

  int ctrl = (glutGetModifiers() & GLUT_ACTIVE_CTRL) != 0;
  if (ctrl) {
    /* When Ctrl is held, GLUT may deliver a control char (1..26) or the
     * raw letter; normalize both to a lowercase letter. */
    int c = (key >= 1 && key <= 26) ? (key + 'a' - 1) : tolower(key);
    switch (c) {
      case 'x':                                   /* exit              */
        shutdownAndExit();
        break;
      case 'p':                                   /* pause / unpause   */
        g_pausemode ^= 1;
        g_singlestep = 0;
        break;
      case 'o':                                   /* single step       */
        if (g_pausemode) g_singlestep = 1;
        break;
      case 't':                                   /* toggle textures   */
        dsSetTextures(dsGetTextures() ^ 1);
        break;
      case 's':                                   /* toggle shadows    */
        dsSetShadows(dsGetShadows() ^ 1);
        break;
      case 'v': {                                 /* print viewpoint   */
        float xyz[3], hpr[3];
        dsGetViewpoint(xyz, hpr);
        printf("Viewpoint = (%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)\n",
               xyz[0], xyz[1], xyz[2], hpr[0], hpr[1], hpr[2]);
        break;
      }
      default:
        break;
    }
    return;
  }

  /* normal key -> user command callback */
  if (g_fn && g_fn->command) g_fn->command(key);
}

static void mouseCB(int button, int state, int x, int y)
{
  int bit = 0;
  if      (button == GLUT_LEFT_BUTTON)   bit = 1;
  else if (button == GLUT_MIDDLE_BUTTON) bit = 2;
  else if (button == GLUT_RIGHT_BUTTON)  bit = 4;

  if (state == GLUT_DOWN) g_mouse_mode |=  bit;
  else                    g_mouse_mode &= ~bit;

  g_mouse_x = x;
  g_mouse_y = y;
}

static void motionCB(int x, int y)
{
  /* Feed deltas to the core camera controller. dsMotion's mode bits match
   * ours: 1=left (pan/tilt), 2=middle, 4=right (dolly/strafe). */
  dsMotion(g_mouse_mode, x - g_mouse_x, y - g_mouse_y);
  g_mouse_x = x;
  g_mouse_y = y;
}

/* ----------------------------------------------------------------------- */
/* the one function the core calls into us (internal.h)                    */
/* ----------------------------------------------------------------------- */

void dsPlatformSimLoop(int window_width, int window_height,
                       dsFunctions *fn, int initial_pause)
{
  g_fn        = fn;
  g_width     = window_width;
  g_height    = window_height;
  g_pausemode = initial_pause;

  /* glutInit needs an argv; dsSimulationLoop already consumed the real one
   * (parsing -notex/-pause/...), so a placeholder is fine here. */
  int   ac  = 1;
  char  a0[] = "ode-buggy-ramp";
  char *av[] = { a0, 0 };
  glutInit(&ac, av);

  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
  glutInitWindowSize(window_width, window_height);
  glutCreateWindow("ODE buggy over a ramp  -  DrawStuff / GLUT");

  glutDisplayFunc(displayCB);
  glutReshapeFunc(reshapeCB);
  glutKeyboardFunc(keyboardCB);
  glutMouseFunc(mouseCB);
  glutMotionFunc(motionCB);
  glutTimerFunc(16, timerCB, 0);

  fprintf(stderr,
    "\n"
    "DrawStuff (GLUT backend) v%d.%02d\n"
    "   Ctrl-P : pause / unpause      Ctrl-O : single step (while paused)\n"
    "   Ctrl-T : toggle textures      Ctrl-S : toggle shadows\n"
    "   Ctrl-V : print viewpoint      Ctrl-X / ESC : exit\n"
    "   Drag in the window to move the camera:\n"
    "     left = pan/tilt   right = forward/strafe   middle = strafe/up\n"
    "\n",
    DS_VERSION >> 8, DS_VERSION & 0xff);

  /* Load textures + GL state, then let the demo set its viewpoint etc. */
  dsStartGraphics(window_width, window_height, fn);
  if (fn->start) fn->start();

  glutMainLoop();          /* does not return on Apple GLUT */
  /* If a future/freeglut build ever returns from the loop, finish cleanly: */
  shutdownAndExit();
}
