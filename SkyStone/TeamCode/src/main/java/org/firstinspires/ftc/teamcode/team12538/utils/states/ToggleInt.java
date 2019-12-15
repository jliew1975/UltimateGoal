package org.firstinspires.ftc.teamcode.team12538.utils.states;

public class ToggleInt {
    private int toggle; //Current state of the toggle
    private boolean previouslyPressed = false; //Was the input device (whatever is passed to input) true last time?
    private final int maxToggle; //the number of states that you want to toggle through

    /**
     *
     * @param maxToggle the number of states that you want to toggle through
     */
    public ToggleInt(int maxToggle) {   //Pass this the number of values you want the toggle to have. When the toggle is about to reach this number, it will go back to 0.
        this(maxToggle, 0);
    }

    public ToggleInt(int maxToggle, int startingToggle) {
        this.maxToggle = maxToggle;
        toggle = startingToggle;
    }

    /**
     * Call this method in your main op-mode loop
     * @param currentlyPressed the state of whatever actuator (eg. button) you want to use to control the toggle
     */
    public void input(boolean currentlyPressed) {
        //if the button is being pressed now and wasn't being pressed last time, change the value of the toggle
        if (currentlyPressed && !previouslyPressed) toggle();
        //change previoulyPressed to reflect the current state of the toggle
        previouslyPressed = currentlyPressed;
    }

    /**
     * Where the actual toggling happens. Generally you shouldn't need this
     */
    public void toggle() {
        toggle++;
        toggle %= maxToggle;
    }

    /**
     *
     * @param toggle the current state of the toggle
     */
    public void setToggle(int toggle) { //Set the toggle to a specific value (generally you shouldn't need this)
        this.toggle = toggle;
    }

    /**
     *
     * @return the current state of the toggle
     */
    public int output() {   //Get the value of the toggle; call this whenever you need to use the toggle to control something
        return toggle;
    }
}
