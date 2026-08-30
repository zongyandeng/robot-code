package frc.robot;

public final class Constants {
    public final class ShooterConstants {
        public static final int kTopShooterId = 0;
        public static final int kBottomShooterId = 0;
        
        public static final double kShooterVelocityVoltage = 0;
        public static final double kShooterDutyCycleOut = 0;

        public static final double kShooterkP = 0.11;
        public static final double kShooterkI = 0.0;
        public static final double kShooterkD = 0.005;
        public static final double kShooterkV = 0.12;

        public static final double kShooterStatorCurrentLimit = 40.0;
        public static final boolean kShooterStatorCurrentLimitEnable = true;

        public static final double kShooterSupplyCurrentLimit = 35.0;
        public static final double kShooterSupplyCurrentLowerLimit = 30.0;
        public static final double kShooterSupplyCurrentLowerTime = 1.0;
        public static final boolean kShooterSupplyCurrentLimitEnable = true;

        public static final double kShooterVoltageClosedLoopRampPeriod = 0.25;
    }

    public final class TurretConstants {
        public static final int kTurretId = 0;

        public static final double kTurretMotionMagicVoltage = 0;

        public static final double kTurretkP = 12.0;
        public static final double kTurretkI = 0.0;
        public static final double kTurretkD = 0.1;

        public static final double kTurretMotionMagicCruiseVelocity = 10;
        public static final double kTurretMotionMagicAcceleration = 20;
        public static final double kTurretdMotionMagicJerk = 0;

        public static final double kTurretStatorCurrentLimit = 40.0;
        public static final boolean kTurretStatorCurrentLimitEnable = true;

        public static final double kTurretSupplyCurrentLimit = 35.0;
        public static final double kTurretSupplyCurrentLowerLimit = 30.0;
        public static final double kTurretSupplyCurrentLowerTime = 1.0;
        public static final boolean kTurretSupplyCurrentLimitEnable = true;


    }

    public final class HoodConstants {
        public static final int kHoodId = 0;

        public static final double kHoodMotionMagicVoltage = 0;

        public static final double kHoodkP = 12.0;
        public static final double kHoodkI = 0.0;
        public static final double kHoodkD = 0.1;

        public static final double kHoodMotionMagicCruiseVelocity = 10;
        public static final double kHoodMotionMagicAcceleration = 20;
        public static final double kHoodMotionMagicJerk = 0;

        public static final double kHoodStatorCurrentLimit = 40.0;
        public static final boolean kHoodStatorCurrentLimitEnable = true;

        public static final double kHoodSupplyCurrentLimit = 35.0;
        public static final double kHoodSupplyCurrentLowerLimit = 30.0;
        public static final double kHoodSupplyCurrentLowerTime = 1.0;
        public static final boolean kHoodSupplyCurrentLimitEnable = true;
        
    }

    public final class FeederConstants {
        public static final int kFeederId = 0;

        public static final double kFeederMotionMagicVoltage = 0;

        public static final double kFeederRotationPerStep = 12.0 * (60.0 / 360.0);

        public static final double kFeederkP = 15.0;
        public static final double kFeederkI = 0.0;
        public static final double kFeederkD = 0.1;

        public static final double kFeederMotionMagicCruiseVelocity = 15;
        public static final double kFeederMotionMagicAcceleration = 30;
        public static final double kFeederMotionMagicJerk = 0;

        public static final double kFeederStatorCurrentLimit = 40.0;
        public static final boolean kFeederStatorCurrentLimitEnable = true;

        public static final double kFeederSupplyCurrentLimit = 35.0;
        public static final double kFeederSupplyCurrentLowerLimit = 30.0;
        public static final double kFeederSupplyCurrentLowerTime = 1.0;
        public static final boolean kFeederSupplyCurrentLimitEnable = true;
        
    }
}
