package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "see", group = "auton")
public class see extends LinearOpMode{
    //robot boWei = new robot();
    VuforiaLocalizer vuforia = null;
    TFObjectDetector tfod;
    final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    final String LABEL_FIRST_ELEMENT = "Quad";
    final String LABEL_SECOND_ELEMENT = "Single";
    public void initTF(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AZrZbhj/////AAAAmQF53NIZpEu8twb/VTB2FVWHbBhbMfb3tV+BUuyjHQKpKVPMUg/QLWJr3ZruBFCpSZOIe4Ss5JD6EuVJdnIksAIDC0sxSf9s3JX/QadoDOgzRLZgWF8E19KI6tUD5Anf1QBJNfnzIrrySGgjGyW7GgEvdCzfElATS8DnKN2LY3Fb0+kVp8mrKcEy7SpxUzmllclnYwDXBtkr2e5ad0sbG0JFZkH5A7OMzBZnJFqDAGCqLJLecP5251x+YhJnAK44NKMI3iGHZOZyk318nV6OjEsnPzVf3lSoQ7Ly34cEObjyLbXt9bVsR0fV1ahaW2MrVOeTNF2WniBnHus6IIq6cpL677z7ZmR6NuXtZs8YJQyw";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public void runOpMode(){
        //boWei.init(hardwareMap, this);
        waitForStart();
        initTF();
            if (tfod != null){
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null){
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals("Single")){
                            //stuff
                        }
                        else if (recognition.getLabel().equals("Quad")){
                            //stuff
                        }
                        else{
                            //stuff
                        }
                    }
                }
        }
    }


}
