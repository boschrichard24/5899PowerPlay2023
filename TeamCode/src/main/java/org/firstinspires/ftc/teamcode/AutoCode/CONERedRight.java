package org.firstinspires.ftc.teamcode.AutoCode;

//-----imports-----
//main imports
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//opencv imports
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//webcam imports

//                  O R I N G I N A L   A U T O   W O K I N G                   \\

@Autonomous(name="CONERedRight Auto", group="CompetitionAuto")
//@Disabled
public class CONERedRight extends AutoSupplies {
    @Override
    public void runOpMode() {
        int daWay = 0;
        //  Establish all hardware
        initForAutonomous();
        //  Wait until start
        //waitForStart();
        initVision();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        int path = 0;

        Recognition cone = null;
        long halfSec = 700;
        runtime.reset();
        while (runtime.milliseconds() <= halfSec) {
            cone = getConeType();
            if (cone != null) {
                break;
            }
        }
        path = getZone(cone);
        telemetry.addLine("Path: " + path);
        telemetry.update();

        sleep(300);
        if (path == 1) {
            daWay = 1;
        } else if (path == 2) {
            daWay = 2;
        } else if(path == 3){
            daWay = 3;
        } else{ lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED); }
        sleep(300);

        if (daWay == 1) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

            encoderMove(1000, -1, 0);
            pause(800);
            encoderMove(1200, 0, 1);
            pause(800);
            turnToS(-44,0.5,2);
            ArmUP();
            setArmEncoderMode();
            telemetry.addData("LEFT Arm Vals :: ", liftLeft.getCurrentPosition());
            telemetry.addData("RIGHT Arm Vals :: ", liftRight.getCurrentPosition());
            telemetry.update();
            //pause(2000);
            encoderMove(390, 0, 0.3);
            pause(1000);
            Release();
            pause(500);
            telemetry.addData("DOWN TIME:: ", 67);
            telemetry.update();
            encoderMove(200, 0, -0.3);
            ArmDOWN();
            pause(2000);
            turnToS(18,0.5,2);
            telemetry.addData("After break", 36);
            telemetry.update();

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
        } else if (daWay == 2) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

            encoderMove(150, -0.3, 0.3);
            pause(800);
            encoderMove(725, 0, 0.8);
            pause(800);
            turnToS(44,0.5,2);
            ArmUP();
            setArmEncoderMode();
            telemetry.addData("LEFT Arm Vals :: ", liftLeft.getCurrentPosition());
            telemetry.addData("RIGHT Arm Vals :: ", liftRight.getCurrentPosition());
            telemetry.update();
            //pause(2000);
            encoderMove(315, 0, 0.3);
            pause(500);
            Release();
            pause(500);
            telemetry.addData("DOWN TIME:: ", 67);
            telemetry.update();
            encoderMove(315, 0, -0.3);
            ArmDOWN();
            pause(2000);
            turnToS(-20,0.5,2);
            telemetry.addData("After break", 56);
            telemetry.update();

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
        } else if(daWay == 3){
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

            encoderMove(200, -0.5, 0.5);
            pause(800);
            encoderMove(1000, 0, 1);
            pause(800);
            turnToS(44,0.5,2);
            ArmUP();
            setArmEncoderMode();
            telemetry.addData("LEFT Arm Vals :: ", liftLeft.getCurrentPosition());
            telemetry.addData("RIGHT Arm Vals :: ", liftRight.getCurrentPosition());
            telemetry.update();
            //pause(2000);
            encoderMove(390, 0, 0.3);
            pause(1000);
            Release();
            pause(500);
            telemetry.addData("DOWN TIME:: ", 67);
            telemetry.update();
            encoderMove(200, 0, -0.3);
            ArmDOWN();
            pause(2000);
            turnToS(-75,0.5,2);
            pause(2000);
            encoderMove(650, 0, 1);
            telemetry.addData("After break", 86);
            telemetry.update();

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE);
        } else{
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            encoderMove(200, -0.5, 0.5);
            pause(800);
            encoderMove(1000, 0, 1);
            pause(800);
            turnToS(44,0.5,2);
            ArmUP();
            setArmEncoderMode();
            telemetry.addData("LEFT Arm Vals :: ", liftLeft.getCurrentPosition());
            telemetry.addData("RIGHT Arm Vals :: ", liftRight.getCurrentPosition());
            telemetry.update();
            //pause(2000);
            encoderMove(390, 0, 0.3);
            pause(1000);
            Release();
            pause(500);
            telemetry.addData("DOWN TIME:: ", 678000);
            telemetry.update();
            encoderMove(200, 0, -0.3);
            ArmDOWN();
            pause(2000);
            turnToS(-20,0.5,2);
            telemetry.addData("After break", 678000);
            telemetry.update();

            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
        }
        //DONE
        if(daWay != 0) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE);
        }

    }
}