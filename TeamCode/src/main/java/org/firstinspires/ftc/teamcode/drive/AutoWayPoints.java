package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class AutoWayPoints {

    public enum StackPosition {
        WALL(new Pose2d(-56.5, -33.2, new Rotation2d(0)), new com.arcrobotics.ftclib.geometry.Pose2d(-57.5, 33.35, new Rotation2d(0))),
        CENTER(new Pose2d(-56.5, 12, new Rotation2d(0)), new Pose2d(-57.5, 33.35, new Rotation2d(0)));

        public final Pose2d red, blue;

        StackPosition(Pose2d red, Pose2d blue) {
            this.red = red;
            this.blue = blue;
        }
    }

    public enum DropPosition {
        WALL(new Pose2d(52.3, -37, new Rotation2d(0)), new Pose2d(52.0, 39.8, new Rotation2d(0))),
        CENTER(new Pose2d(52.3, -12, new Rotation2d(0)), new Pose2d(52.0, 12, new Rotation2d(0)));

        public final Pose2d red, blue;

        DropPosition(Pose2d red, Pose2d blue) {
            this.red = red;
            this.blue = blue;
        }
    }

    public enum CrossStackSide {
        WALL(new Pose2d(-33, -57, new Rotation2d(0)), new Pose2d(-33, 57, new Rotation2d(0))),
        CENTER(new Pose2d(-33, -9, new Rotation2d(0)), new Pose2d(-33, 9, new Rotation2d(0)));

        public final Pose2d red, blue;

        CrossStackSide(Pose2d red, Pose2d blue) {
            this.red = red;
            this.blue = blue;
        }
    }

    public enum CrossBackdropSide {
        WALL(new Pose2d(33, -57, new Rotation2d(0)), new Pose2d(33, 57, new Rotation2d(0))),
        CENTER(new Pose2d(33, -9, new Rotation2d(0)), new Pose2d(33, 9, new Rotation2d(0)));

        public final Pose2d red, blue;

        CrossBackdropSide(Pose2d red, Pose2d blue) {
            this.red = red;
            this.blue = blue;
        }
    }

    public enum ParkPosition {
        BACK(new Pose2d(48, -37, new Rotation2d(0)), new Pose2d(48, 39.8, new Rotation2d(0))),
        WALL(new Pose2d(48, -58, new Rotation2d(0)), new Pose2d(48, 58, new Rotation2d(0))),
        CENTER(new Pose2d(48, -8, new Rotation2d(0)), new Pose2d(48, 8, new Rotation2d(0)));

        public final Pose2d red, blue;

        ParkPosition(Pose2d red, Pose2d blue) {
            this.red = red;
            this.blue = blue;
        }
    }


}
