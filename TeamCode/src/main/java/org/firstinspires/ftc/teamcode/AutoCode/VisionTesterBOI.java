package org.firstinspires.ftc.teamcode.AutoCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//if ye have any Qs consult the concept version of this in FTCRobotController/exernal.samples place thing

@TeleOp(name = "VisionTesterBOI", group = "Working")
//@Disabled
public class VisionTesterBOI extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "PowerPlayVisionV1.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "image1",
            "image2",
            "image3"
    };

    private static final String VUFORIA_KEY =
            "AcuKVGn/////AAABmYltM2e7pk7PgntBn1QBuWhrTu52XxsTTE0NUewmsJRB/KPGYohT+y+YHK8/LEcyIki6iW/Msvl8c4kwg5XFMGWY19pEmKxhqgtXd58d5Kyu+UC8NFYlEf52vluc0yibUUmianPhos+tutELIA2PqQqSgM3WTXxH+fwUdPIvAJuTHZqxu9t9cK3qKZeLzsgDSOXeKEjOcmkJQsFsneppUGPTrehHCDLRYyOAwDgHsr2X65ZhZ4cOSYoQR0WDo2ogRvFxuWDPinD9Gj8jq4AxNDrAr3kLkZwbtSc6IfD0dAysXtIoK41JgDZNK2BG93C9tDoNmNTUJNFv3pn6+cywIch5e1ylQ0ZBfZ47ZMCTDJA+";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        //Activate TensorFlow Object Detection first so Camera window TensorFlow annotations visible
        if (tfod != null) {
            tfod.activate();
            // < magnification = higher accuracy at < dist. keep as low as posible
            tfod.setZoom(1.0, 22.0 / 9.0); // 16/9
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.35f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}