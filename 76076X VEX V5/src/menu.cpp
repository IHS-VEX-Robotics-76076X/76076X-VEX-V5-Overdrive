#include "menu.hpp"

// No PROS include here on purpose - this file is pure logic so the host
// tests can exercise it directly. See the header for the reasoning.

const char* auton_name(AutonRoutine routine) {
    switch (routine) {
        case AutonRoutine::RED_CLOSE:  return "RED CLOSE";
        case AutonRoutine::RED_FAR:    return "RED FAR";
        case AutonRoutine::BLUE_CLOSE: return "BLUE CLOSE";
        case AutonRoutine::BLUE_FAR:   return "BLUE FAR";
        case AutonRoutine::SKILLS:     return "SKILLS";
        case AutonRoutine::SAFE:       return "SAFE (no move)";
    }
    return "?";
}

// Next routine in the list, wrapping round at the end so you can always
// reach every option with the one button.
static AutonRoutine next_routine(AutonRoutine routine) {
    int index = static_cast<int>(routine) + 1;
    if (index >= AUTON_ROUTINE_COUNT) index = 0;
    return static_cast<AutonRoutine>(index);
}

MenuState menu_press(MenuState state, MenuButton button) {
    switch (state.screen) {
        case Screen::HOME:
            // One screen per button, which is the whole reason the three
            // buttons do different things from here.
            if (button == MenuButton::LEFT)   state.screen = Screen::AUTON_SELECT;
            if (button == MenuButton::CENTER) state.screen = Screen::TEST_MENU;
            if (button == MenuButton::RIGHT)  state.screen = Screen::STATUS;
            break;

        case Screen::AUTON_SELECT:
            // Picking the routine for a real match. Confirm and cancel both
            // land back on HOME - driver control was never interrupted, so
            // "back to HOME" is the same thing as "back to driving".
            if (button == MenuButton::LEFT)   state.selected = next_routine(state.selected);
            if (button == MenuButton::CENTER) state.screen = Screen::HOME; // confirm
            if (button == MenuButton::RIGHT)  state.screen = Screen::HOME; // cancel
            break;

        case Screen::TEST_MENU:
            // Same selection, but CENTER runs the route here and now instead
            // of saving it for the match. Stays on this screen afterwards so
            // you can tweak and run again without renavigating.
            if (button == MenuButton::LEFT)   state.selected = next_routine(state.selected);
            if (button == MenuButton::CENTER) state.runTestNow = true;
            if (button == MenuButton::RIGHT)  state.screen = Screen::HOME;
            break;

        case Screen::STATUS:
            // Read-only page: any button backs out. Nothing here to select,
            // so giving the buttons separate jobs would just be a trap.
            state.screen = Screen::HOME;
            break;
    }
    return state;
}
