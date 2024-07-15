package frc.robot;

public class Constants {
    public static class VisionConstants{
        public static final String LIMELIGHT_NAME = "limelight";
    }
    public static class IntakeConstants{
        public static final int INTAKE_TOPROLLER_ID = 20;
        public static final int INTAKE_BOTTOMROLLER_ID = 21;
        public static final int INTAKE_BEAMBREAK_ID = 0;
    }
    public static class ShooterConstants{
        public static final int SHOOTER_TOPFLYWHEEL_ID = 30;
        public static final int SHOOTER_BOTTOMFLYWHEEL_ID = 31;
        public static final int SHOOTER_BOTTOMFEEDER_ID = 32;
        public static final int SHOOTER_TOPFEEDER_ID = 33;
        public static final int SHOOTER_PIVOTMOTOR_ID = 35;
        public static final double SHOOTER_TIME_TO_SCORE = 0.4;

        //speaker shot 
    }
    public static class ClimberConstants{
        public static final int CLIMBER_LEFT_ID = 40;
        public static final int CLIMBER_RIGHT_ID = 41;
        // dutycycles will need to be tuned to each climber to make them have the same speed
        public static final double CLIMBER_LEFT_DUTYCYCLE = 0.75;
        public static final double CLIMBER_RIGHT_DUTYCYCLE = 0.75;
    }
    public static final class FieldConstants{
        public static final int SPEAKER_HEIGHT = 7 * 12;
        public static final double SPEAKER_X_BLUE = 0;
        public static final double SPEAKER_X_RED = 54;
      }


}
