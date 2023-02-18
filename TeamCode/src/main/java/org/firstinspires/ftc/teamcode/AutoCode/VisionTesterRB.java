package org.firstinspires.ftc.teamcode.AutoCode;
//-----imports-----
//main imports
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//opencv imports
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutoCode.NEWcamera;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
//webcam imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name="VisionTesterRB", group="CompetitionAuto")
//@Disabled
public class VisionTesterRB extends NEWcamera{
    @Override
    public void runOpMode() {

        //  Establish all hardware

        initForAutonomous();


        //  Wait until start
        waitForStart();
        //basketServoUp(); -- tried moving this down lower. see how it works before deleting this
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY);
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();
        sleep(300);
        int ringCnt = pipeline.getAnalysis(); // 4 rings: >150 --- 1 ring: >130 & <150 --- 0 rings: <130
        sleep(300);
        if (ringCnt <= 130) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (ringCnt > 130 && ringCnt < 150) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        encoderMove(500, 0, .7);
        turnToS(0, .6, 2);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE);
        turnToS(0, .6, 2);
        if (ringCnt > 130 && ringCnt < 150) {
            encoderMove(700, 0, .4);
        }
        else if(ringCnt >= 150){
            encoderMove(500, 0, .4);
        }

        setPower(0, 0);
        if (ringCnt > 130 && ringCnt < 150) {
            turnToS(90, .7, 2);
            encoderMove(400, 0, -0.5);
        } else if (ringCnt >= 150) {
            turnToS(210, .7, 2);
        }
        setPower(0,0);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);
        sleep(400);
        sleep(200);
        if (ringCnt > 130 && ringCnt < 150) {
            encoderMove(200, 0, .5);
        }
        sleep(500);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON);
        if(ringCnt > 150){
            turnToS(360, .7, 2);
            resetAngle();
        }
        else {
            turnToS(0, .7, 2);
        }
        if (ringCnt > 150) {
            encoderMove(500, -0.5, 0);
            turnToS(0, .6, 2);
            encoderMove(200, 0, .5);
            turnToS(0, .6, 2);
        } else {
            encoderMove(200, 0, .5);
            turnToS(0, .6, 2);
            encoderMove(700, -0.5, 0);
            turnToS(0, .6, 2);
        }

        setPower(0,.3);
        sleep(800);//was not updated to the bot yet... was at 500
        setPower(0,0);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);

        encoderMove(300, 0, -.3);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
        if(ringCnt > 130 && ringCnt < 150){
            encoderMove(1400, 0, .6);
        }
        turnToS(0, .7, 2);
        encoderMove(950, 0, -1);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
        //  Turn all motors off and sleep
        setPower(0, 0);
        sleep(1000);
    }
}