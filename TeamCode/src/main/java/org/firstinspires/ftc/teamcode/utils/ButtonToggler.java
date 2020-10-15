package org.firstinspires.ftc.teamcode.utils;

/**
 * A class used for tracking advanced button values.
 */
public class ButtonToggler {

    private boolean on;
    private boolean mode;
    private double duration;

    public ButtonToggler() {
        on = false;
        mode = false;
        duration = 0;
    }

    /**
     * Toggles the mode if the button was just pressed.
     * @param current The current value of the toggling button.
     */
    public boolean ifPress (boolean current) {
        if (!on && current) {
            mode = !mode;
            return true;
        }

        return false;
    }

    /**
     * Toggles the mode if the button was just released.
     * @param current The current value of the toggling button.
     */
    public boolean ifRelease (boolean current) {
        if (on && !current) {
            mode = !mode;
            return true;
        }

        return false;
    }

    /**
     * Updates the button's last value to the new value.
     * &nbsp; This should be called at the end of each call of init.
     * @param current The current value of the toggling button.
     */
    public void update (boolean current) {
        on = current;
    }

    /**
     * Returns the current mode of the ButtonToggler.
     * @return The current mode of the ButtonToggler.
     */
    public boolean getMode () {
        return mode;
    }

}