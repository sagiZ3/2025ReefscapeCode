package frc.lib.generic.hardware;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class KeyboardController {
    public KeyboardController() {
        String[] keys = {
                "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "zero", "minus", "equals",
                "q", "w", "e", "r", "t", "y", "u", "i", "o", "p", "a", "s", "d", "f", "g", "h", "j", "k", "l", "z", "x", "c", "v", "b", "n", "m",
                "numpad0", "numpad1", "numpad2", "numpad3", "numpad4", "numpad5", "numpad6", "numpad7", "numpad8", "numpad9"
        };
    }

    private Trigger createKeyTrigger(String key) {
        return new Trigger(new LoggedNetworkBoolean("/SmartDashboard/keyboard/" + key, false)::get);
    }

    public Trigger one() {
        return createKeyTrigger("1");
    }

    public Trigger two() {
        return createKeyTrigger("2");
    }

    public Trigger three() {
        return createKeyTrigger("3");
    }

    public Trigger four() {
        return createKeyTrigger("4");
    }

    public Trigger five() {
        return createKeyTrigger("5");
    }

    public Trigger six() {
        return createKeyTrigger("6");
    }

    public Trigger seven() {
        return createKeyTrigger("7");
    }

    public Trigger eight() {
        return createKeyTrigger("8");
    }

    public Trigger nine() {
        return createKeyTrigger("9");
    }

    public Trigger zero() {
        return createKeyTrigger("0");
    }

    public Trigger q() {
        return createKeyTrigger("q");
    }

    public Trigger w() {
        return createKeyTrigger("w");
    }

    public Trigger e() {
        return createKeyTrigger("e");
    }

    public Trigger r() {
        return createKeyTrigger("r");
    }

    public Trigger t() {
        return createKeyTrigger("t");
    }

    public Trigger y() {
        return createKeyTrigger("y");
    }

    public Trigger u() {
        return createKeyTrigger("u");
    }

    public Trigger i() {
        return createKeyTrigger("i");
    }

    public Trigger o() {
        return createKeyTrigger("o");
    }

    public Trigger p() {
        return createKeyTrigger("p");
    }

    public Trigger a() {
        return createKeyTrigger("a");
    }

    public Trigger s() {
        return createKeyTrigger("s");
    }

    public Trigger d() {
        return createKeyTrigger("d");
    }

    public Trigger f() {
        return createKeyTrigger("f");
    }

    public Trigger g() {
        return createKeyTrigger("g");
    }

    public Trigger h() {
        return createKeyTrigger("h");
    }

    public Trigger j() {
        return createKeyTrigger("j");
    }

    public Trigger k() {
        return createKeyTrigger("k");
    }

    public Trigger l() {
        return createKeyTrigger("l");
    }

    public Trigger z() {
        return createKeyTrigger("z");
    }

    public Trigger x() {
        return createKeyTrigger("x");
    }

    public Trigger c() {
        return createKeyTrigger("c");
    }

    public Trigger v() {
        return createKeyTrigger("v");
    }

    public Trigger b() {
        return createKeyTrigger("b");
    }

    public Trigger n() {
        return createKeyTrigger("n");
    }

    public Trigger m() {
        return createKeyTrigger("m");
    }

    public Trigger numpad0() {
        return createKeyTrigger("numpad0");
    }

    public Trigger numpad1() {
        return createKeyTrigger("numpad1");
    }

    public Trigger numpad2() {
        return createKeyTrigger("numpad2");
    }

    public Trigger numpad3() {
        return createKeyTrigger("numpad3");
    }

    public Trigger numpad4() {
        return createKeyTrigger("numpad4");
    }

    public Trigger numpad5() {
        return createKeyTrigger("numpad5");
    }

    public Trigger numpad6() {
        return createKeyTrigger("numpad6");
    }

    public Trigger numpad7() {
        return createKeyTrigger("numpad7");
    }

    public Trigger numpad8() {
        return createKeyTrigger("numpad8");
    }

    public Trigger numpad9() {
        return createKeyTrigger("numpad9");
    }
}
