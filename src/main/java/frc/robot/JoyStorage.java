package frc.robot;

import java.io.Serializable;

public class JoyStorage implements Serializable {

    //public String name;
    //public String email;
   // private String[] roles;
    //public boolean admin;
    /*
    double lefty;
    double righty;
    boolean buttonA;
    */

        public double leftYstick;  //left y stick
		public double rightYstick;  //right y stick
        public boolean buttonlb; // back intake
        public boolean buttonrb;// front intake 
        public boolean buttonX; // shooooot high
        public boolean buttonReleaseX; // shoot high off
        public boolean buttonReleaserb; // turn off intake

    public JoyStorage() {
    }

    public JoyStorage(
        double leftYstick,
        double rightYstick,
        boolean buttonlb,
        boolean buttonrb,
        boolean buttonX,
        boolean buttonReleaseX,
        boolean buttonReleaserb) {
        this.leftYstick = leftYstick;
        this.rightYstick = rightYstick;
        this.buttonlb = buttonlb;
        this.buttonrb = buttonrb;
        this.buttonX = buttonX;
        this.buttonReleaseX = buttonReleaseX;
        this.buttonReleaserb = buttonReleaserb;

    }

    // getters and setters, toString() .... (omitted for brevity)
}