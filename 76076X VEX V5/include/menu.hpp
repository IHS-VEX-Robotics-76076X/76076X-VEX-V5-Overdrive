#pragma once

// ===========================================================================
//                        BRAIN SCREEN MENUS
// ===========================================================================
//
// The brain has three buttons under the screen and eight lines of text. This
// drives a small menu system on them:
//
//   HOME          LEFT  -> pick the match auton    (AUTON_SELECT)
//                 MID   -> test auton routes       (TEST_MENU)
//                 RIGHT -> what's plugged in       (STATUS)
//
//   AUTON_SELECT  LEFT  -> next route
//                 MID   -> confirm, back to HOME (driver control carries on)
//                 RIGHT -> cancel, back to HOME
//
//   TEST_MENU     LEFT  -> next route
//                 MID   -> RUN the selected route right now
//                 RIGHT -> back to HOME / driver control
//
//   STATUS        any   -> back to HOME
//
// WHY THE STATE MACHINE LIVES HERE, AWAY FROM THE SCREEN CODE
//
// Button callbacks run in their own PROS task, and drawing needs real
// hardware, so neither can be tested on a laptop. The part that actually has
// bugs in it - which button goes where, does the selection wrap, does a test
// request get cleared - is pure data with no hardware in it, so it lives
// here and is covered by tests/test_menu.cpp.
//
// ===========================================================================

// The autonomous routines, in the order the menu cycles them.
enum class AutonRoutine {
    RED_CLOSE,
    RED_FAR,
    BLUE_CLOSE,
    BLUE_FAR,
    SKILLS,
    SAFE
};

// How many entries AutonRoutine has. Kept next to the enum so adding a
// routine means changing both lines together, right here.
constexpr int AUTON_ROUTINE_COUNT = 6;

// Short label for the screen. Lines are ~32 characters, so keep these brief.
const char* auton_name(AutonRoutine routine);

enum class Screen {
    HOME,
    AUTON_SELECT,
    TEST_MENU,
    STATUS
};

enum class MenuButton { LEFT, CENTER, RIGHT };

struct MenuState {
    Screen screen = Screen::HOME;

    // The routine autonomous() will run in a real match.
    AutonRoutine selected = AutonRoutine::RED_CLOSE;

    // Set when the driver asks to run a route from the test menu. opcontrol
    // picks this up, runs the routine, and clears it - see the comment on
    // menu_press() below for why it has to happen that way round.
    bool runTestNow = false;
};

// Applies one button press and returns the new state.
//
// Pure on purpose: no motors, no screen, no globals. The caller owns the
// state and decides what to do with the result, which is what makes the
// whole menu testable without a robot.
//
// runTestNow is a REQUEST, not an action. The routines drive the robot, and
// the only task allowed to do that is the one already driving it - opcontrol.
// A button callback runs in a different task, so if it drove the robot
// itself it would be fighting opcontrol for the same motors every 20ms.
// Instead the callback raises this flag and opcontrol services it.
MenuState menu_press(MenuState state, MenuButton button);
