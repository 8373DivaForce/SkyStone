package org.firstinspires.ftc.teamcode.Functions.Old;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

//class used to initialize vuforia with all the of their positions
//most of this came from the ConceptVuforiaSkyStoneNavigationWebcam class
//but was modified in order to fit in as an initializer
public class InitSkystoneVuforia {

    //initialize class variables
    private final VuforiaLocalizer vuforia;
    private VuforiaTrackables SkystoneTrackables;
    private ArrayList<VuforiaTrackable> SkystoneVuMarks = new ArrayList<>();

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //uses the vuforiaData constructor class to return the initialized data
    public vuforiaData getVuforia() {
        return new vuforiaData(vuforia,SkystoneTrackables, SkystoneVuMarks);
    }
    //takes the hardwaremap, webcam, vuforiakey, and displacements in order to initialize vuforia
    public InitSkystoneVuforia(HardwareMap hMap, WebcamName webcam, String VuforiaKey, float forwardDisplcement, float leftDisplacement, float verticalDisplacement) {

        //get the view ID
        int cameraMonitorViewId = hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());
        //get the localizer parameters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //set the vuforiaLicenseKey
        parameters.vuforiaLicenseKey = VuforiaKey;
        //if the webcam isn't null, tell vuforia to use the webcame
        if (webcam != null) parameters.cameraName = webcam;
        //initialize vuforia
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //retrieve the skystonetrackables
        SkystoneTrackables = vuforia.loadTrackablesFromAsset("Skystone");

        //set their names
        VuforiaTrackable stoneTarget = SkystoneTrackables.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = SkystoneTrackables.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = SkystoneTrackables.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = SkystoneTrackables.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = SkystoneTrackables.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = SkystoneTrackables.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = SkystoneTrackables.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = SkystoneTrackables.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = SkystoneTrackables.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = SkystoneTrackables.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = SkystoneTrackables.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = SkystoneTrackables.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = SkystoneTrackables.get(12);
        rear2.setName("Rear Perimeter 2");

        //add all of them to an arrayList that's easier to iterate through
        SkystoneVuMarks.addAll(SkystoneTrackables);
        //set the field positions of the vuforia trackables
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //set the phones y translation
        phoneYRotate = -180;

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = forwardDisplcement * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = verticalDisplacement * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = leftDisplacement * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : SkystoneVuMarks) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

    }

    //vuforiaData constructor class for retrieving the vuforia data
    public static class vuforiaData {
        public final VuforiaLocalizer vuforia;
        public final VuforiaTrackables trackables;
        public final ArrayList<VuforiaTrackable> vuMarks;
        public vuforiaData(VuforiaLocalizer vuforia, VuforiaTrackables trackables, ArrayList<VuforiaTrackable> vuMarks) {

            this.vuforia = vuforia;
            this.trackables = trackables;
            this.vuMarks = vuMarks;
        }
    }
}
