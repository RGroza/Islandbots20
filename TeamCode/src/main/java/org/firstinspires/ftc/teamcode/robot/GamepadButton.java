package org.firstinspires.ftc.teamcode.robot;

/**
 * Created by KaDon on 8/26/2018.
 */

public class GamepadButton {
    public boolean pressed = false;
    public boolean buttonStatus = false;
    public boolean justPressed = false;
    private int delay;
    private long startTime;
    private boolean isToggled;
    public GamepadButton(int delay, boolean isToggled) {
        this.delay = delay;
        this.isToggled = isToggled;
    }

    public void checkStatus(boolean buttonStatus) {
        if(isToggled) buttonStatus = !buttonStatus;
        this.buttonStatus = buttonStatus;
        if(justPressed) justPressed = false;
        if(buttonStatus && (System.nanoTime()/1000000 - startTime) > delay) {
            startTime = System.nanoTime()/1000000;
            pressed = !pressed;
            justPressed = true;
        }
    }

    public void pressedSwitchStatus() { this.pressed = !this.pressed; }

    public void pressedStatusFalse() {
        this.pressed = false;
    }
}
