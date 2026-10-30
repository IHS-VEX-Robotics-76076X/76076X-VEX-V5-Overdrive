#include <iostream>
#include <cassert>
#include <set>
#include <string>

#include "menu.hpp"

// Convenience: apply a sequence of presses from a fresh HOME state.
static MenuState pressAll(std::initializer_list<MenuButton> buttons) {
    MenuState s;
    for (auto b : buttons) s = menu_press(s, b);
    return s;
}

static void test_home_buttons_open_the_right_screens() {
    std::cout << "[test] HOME: each button opens its own screen\n";

    assert(pressAll({MenuButton::LEFT}).screen == Screen::AUTON_SELECT);
    assert(pressAll({MenuButton::CENTER}).screen == Screen::TEST_MENU);
    assert(pressAll({MenuButton::RIGHT}).screen == Screen::STATUS);

    // HOME itself never starts a routine - only the test menu does.
    assert(pressAll({MenuButton::LEFT}).runTestNow == false);
    assert(pressAll({MenuButton::CENTER}).runTestNow == false);
    assert(pressAll({MenuButton::RIGHT}).runTestNow == false);
}

// The match routine is chosen with one button, so it has to cycle through
// every option and wrap - otherwise some routines are simply unreachable.
static void test_auton_select_reaches_every_routine_and_wraps() {
    std::cout << "[test] AUTON_SELECT: LEFT cycles every routine and wraps\n";

    MenuState s;
    s = menu_press(s, MenuButton::LEFT); // HOME -> AUTON_SELECT

    std::set<std::string> seen;
    seen.insert(auton_name(s.selected));
    for (int i = 0; i < AUTON_ROUTINE_COUNT; i++) {
        s = menu_press(s, MenuButton::LEFT);
        seen.insert(auton_name(s.selected));
    }

    std::cout << "  distinct routines reachable: " << seen.size()
              << " of " << AUTON_ROUTINE_COUNT << "\n";
    assert(static_cast<int>(seen.size()) == AUTON_ROUTINE_COUNT);

    // One extra full lap must land back where it started.
    AutonRoutine before = s.selected;
    for (int i = 0; i < AUTON_ROUTINE_COUNT; i++) s = menu_press(s, MenuButton::LEFT);
    assert(s.selected == before && "cycling a full lap must wrap to the start");
}

// "Pick the route, then carry on driving" - both exits land on HOME, and
// neither may kick off a routine. Starting one here would drive the robot
// during a match while the driver thought they were only choosing.
static void test_auton_select_exits_to_home_without_running_anything() {
    std::cout << "[test] AUTON_SELECT: confirm and cancel both return to HOME, neither runs\n";

    MenuState confirm = pressAll({MenuButton::LEFT, MenuButton::CENTER});
    assert(confirm.screen == Screen::HOME);
    assert(confirm.runTestNow == false && "confirming a match route must not drive the robot");

    MenuState cancel = pressAll({MenuButton::LEFT, MenuButton::RIGHT});
    assert(cancel.screen == Screen::HOME);
    assert(cancel.runTestNow == false);
}

// Confirming keeps the highlighted routine - that selection is what
// autonomous() runs in the match, so losing it would silently run the wrong
// routine.
static void test_confirmed_routine_is_remembered() {
    std::cout << "[test] AUTON_SELECT: the confirmed routine sticks\n";

    MenuState s = pressAll({MenuButton::LEFT, MenuButton::LEFT, MenuButton::LEFT});
    AutonRoutine picked = s.selected;
    s = menu_press(s, MenuButton::CENTER); // confirm

    std::cout << "  picked " << auton_name(picked) << ", after confirm: "
              << auton_name(s.selected) << "\n";
    assert(s.selected == picked);
    assert(s.screen == Screen::HOME);
}

static void test_test_menu_runs_the_selected_routine() {
    std::cout << "[test] TEST_MENU: CENTER requests a run, RIGHT leaves for driver control\n";

    MenuState s = pressAll({MenuButton::CENTER, MenuButton::CENTER});
    assert(s.runTestNow == true && "CENTER in the test menu must request a run");
    assert(s.screen == Screen::TEST_MENU && "stay put so you can run it again");

    // RIGHT is the way back to driving.
    MenuState out = pressAll({MenuButton::CENTER, MenuButton::RIGHT});
    assert(out.screen == Screen::HOME);
    assert(out.runTestNow == false);

    // Selecting in the test menu must not run anything on its own.
    MenuState browsing = pressAll({MenuButton::CENTER, MenuButton::LEFT});
    assert(browsing.runTestNow == false && "browsing must not start a routine");
}

// The status page is read-only, so every button has to get you out. A button
// that did nothing would look like the robot had frozen.
static void test_status_screen_exits_on_any_button() {
    std::cout << "[test] STATUS: any button returns to HOME\n";

    for (auto b : {MenuButton::LEFT, MenuButton::CENTER, MenuButton::RIGHT}) {
        MenuState s = pressAll({MenuButton::RIGHT, b});
        assert(s.screen == Screen::HOME);
        assert(s.runTestNow == false);
    }
}

// auton_name() feeds the screen directly, so a missing case would show "?"
// on the brain mid-competition rather than failing anywhere visible.
static void test_every_routine_has_a_name() {
    std::cout << "[test] every routine has a real name for the screen\n";

    for (int i = 0; i < AUTON_ROUTINE_COUNT; i++) {
        const char* name = auton_name(static_cast<AutonRoutine>(i));
        std::cout << "  " << i << ": " << name << "\n";
        assert(std::string(name) != "?" && "unnamed routine - add it to auton_name()");
        assert(std::string(name).size() <= 20 && "name too long for the LCD line");
    }
}

int main() {
    std::cout << "Host test: Menu\n";

    test_home_buttons_open_the_right_screens();
    test_auton_select_reaches_every_routine_and_wraps();
    test_auton_select_exits_to_home_without_running_anything();
    test_confirmed_routine_is_remembered();
    test_test_menu_runs_the_selected_routine();
    test_status_screen_exits_on_any_button();
    test_every_routine_has_a_name();

    std::cout << "All menu tests passed.\n";
    return 0;
}
