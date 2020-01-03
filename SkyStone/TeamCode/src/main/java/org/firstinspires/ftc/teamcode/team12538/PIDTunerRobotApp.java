package org.firstinspires.ftc.teamcode.team12538;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.apache.commons.lang3.StringUtils;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.team12538.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.drive.PIDMecanumDrive;
import org.firstinspires.ftc.teamcode.team12538.ext.AutoGamepad;
import org.firstinspires.ftc.teamcode.team12538.ext.PIDController;
import org.firstinspires.ftc.teamcode.team12538.ext.PIDControllerV1;
import org.firstinspires.ftc.teamcode.team12538.ext.PIDControllerV2;
import org.firstinspires.ftc.teamcode.team12538.robot.SkyStoneAutoRobot;
import org.firstinspires.ftc.teamcode.team12538.utils.AutoGamepadUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.OpModeUtils;
import org.firstinspires.ftc.teamcode.team12538.utils.states.Button;
import org.firstinspires.ftc.teamcode.team12538.utils.states.ToggleBoolean;
import org.firstinspires.ftc.teamcode.team12538.utils.states.ToggleInt;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Robot PID Tuner", group="Linear Opmode")
public class PIDTunerRobotApp extends RobotApp {
    private double power = 0.3;
    private double kP = 0.0, kI = 0.0, kD = 0.0;
    private double stepP = 1, stepI = 0.01, stepD = 0.01;

    private Button dpadUp = new Button();
    private Button dpadDown = new Button();
    private Button dpadLeft = new Button();
    private Button dpadRight = new Button();

    private Button btnA = new Button();

    private Button bumperLeft = new Button();
    private Button bumperRight = new Button();

    private ToggleBoolean btnB = new ToggleBoolean(true);

    enum ControllerName { ForwardBackwardController , StrafeController, TurnController}

    private ToggleInt selectedPIDComponent = new ToggleInt(3);
    private ToggleInt selectedPIDController = new ToggleInt(3);

    private File case0PIDTunedValuesFile = AppUtil.getInstance().getSettingsFile("Case0PIDTunedValues.txt");
    private File case1PIDTunedValuesFile = AppUtil.getInstance().getSettingsFile("Case1PIDTunedValues.txt");
    private File case2PIDTunedValuesFile = AppUtil.getInstance().getSettingsFile("Case2PIDTunedValues.txt");

    private List<PIDController> controllers = new ArrayList<PIDController>();
    private ControllerName selectedController = ControllerName.values()[selectedPIDController.output()];

    @Override
    public void performRobotOperation() throws InterruptedException {
        try {
            AutoGamepad autoGamepad = new AutoGamepad();

            // Reset encoder values
            OpModeUtils.setResetEncoder(true);

            SkyStoneAutoRobot robot = new SkyStoneAutoRobot();
            robot.init();

            NewMecanumDrive mecanumDrive = (NewMecanumDrive) robot.mecanumDrive;
            String[] KCaptions = new String[] {"KP", "KI", "KD"};
            String[] stepCaptions = new String[] {"stepP", "stepI", "stepD"};

            while(!isStarted() && !isStopRequested()) {
                selectedPIDController.input(gamepad2.b);

                File controllerFile = null;

                controllers.clear();
                controllers.add(mecanumDrive.rotateController);
                selectedController = ControllerName.values()[2];
                controllerFile = case2PIDTunedValuesFile;

                /*
                switch(selectedPIDController.output()) {
                    case 0:
                        controllers.add(mecanumDrive.leftController);
                        controllers.add(mecanumDrive.rightController);
                        selectedController = ControllerName.values()[selectedPIDController.output()];
                        controllerFile = case0PIDTunedValuesFile;
                        break;
                    case 1:
                        controllers.add(mecanumDrive.strafeController);
                        selectedController = ControllerName.values()[selectedPIDController.output()];
                        controllerFile = case1PIDTunedValuesFile;
                        break;
                    case 2:
                        controllers.add(mecanumDrive.rotateController);
                        selectedController = ControllerName.values()[selectedPIDController.output()];
                        controllerFile = case2PIDTunedValuesFile;
                        break;
                }
                */

                // initControllers(controllers, controllerFile);

                telemetry.addData("Selected Controller", selectedController);
                telemetry.addData(KCaptions[0], controllers.get(0).getKP());
                telemetry.addData(KCaptions[1], controllers.get(0).getKI());
                telemetry.addData(KCaptions[2], controllers.get(0).getKD());
                telemetry.update();
            }

            double[] targetPositions = new double[0];;

            while(opModeIsActive()) {
                if(gamepad1.x) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.TurnLeft, power, Math.PI/2);
                    mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if (gamepad1.b) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.TurnRight, power, Math.PI/2);
                    mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.y) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.Forward, 0.5, 60d);
                    // targetPositions = mecanumDrive.calculateTarget(autoGamepad);
                    mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.a) {
                    AutoGamepadUtils.move(autoGamepad, MecanumDrive.AutoDirection.Backward, 0.5, 60d);
                    mecanumDrive.autoNavigateWithGamepad(autoGamepad);
                } else if(gamepad1.right_bumper) {
                    mecanumDrive.resetAngle();
                }

                dpadUp.input(gamepad2.dpad_up);
                dpadDown.input(gamepad2.dpad_down);
                dpadLeft.input(gamepad2.dpad_left);
                dpadRight.input(gamepad2.dpad_right);

                btnA.input(gamepad2.a);
                btnB.input(gamepad2.b);

                bumperLeft.input(gamepad2.left_bumper);
                bumperLeft.input(gamepad2.right_bumper);

                selectedPIDComponent.input(gamepad2.x);

                performPIDAdjustment();

                if(btnB.output()) {
                    KCaptions = new String[] {"KP", "KI", "KD"};
                    stepCaptions = new String[] {"stepP", "stepI", "stepD"};


                    KCaptions[selectedPIDComponent.output()] = "->" + KCaptions[selectedPIDComponent.output()];
                    stepCaptions[selectedPIDComponent.output()] = "->" + stepCaptions[selectedPIDComponent.output()];

                    telemetry.addData("Selected Controller", selectedController);
                    telemetry.addData("Controls", "X to select, A to save");
                    telemetry.addData(KCaptions[0], controllers.get(0).getKP());
                    telemetry.addData(KCaptions[1], controllers.get(0).getKI());
                    telemetry.addData(KCaptions[2], controllers.get(0).getKD());
                    telemetry.addData(stepCaptions[0], stepP);
                    telemetry.addData(stepCaptions[1], stepI);
                    telemetry.addData(stepCaptions[2], stepD);
                    telemetry.addData("Power", power);
                    telemetry.addData("Current angle", mecanumDrive.getAngle());
                    telemetry.addData("right_stick_x", gamepad1.right_stick_x);
                    telemetry.update();
                } else {
                    if(targetPositions != null) {
                        // mecanumDrive.printTelemetry(autoGamepad, targetPositions);
                    }
                }
            }
        } finally {
            OpModeUtils.stop();
        }
    }

    private void initControllers(List<PIDControllerV2> controllers, File pidTunedValuesFile) {
        /*
        String pidValues = ReadWriteFile.readFile(pidTunedValuesFile);
        if(StringUtils.isNotBlank(pidValues)) {
            String[] pids = pidValues.split("|");
            controllers.forEach(c -> {
                c.setKP(Double.parseDouble(StringUtils.trim(pids[0])));
                c.setKI(Double.parseDouble(StringUtils.trim(pids[1])));
                c.setKD(Double.parseDouble(StringUtils.trim(pids[2])));
            });
        }
        */
    }

    private void performPIDAdjustment() {
        if (dpadUp.onPress()) {
            switch (selectedPIDComponent.output()) {
                case 0:
                    controllers.forEach(c -> c.setKP(c.getKP()+stepP));
                    break;
                case 1:
                    controllers.forEach(c -> c.setKI(c.getKI()+stepI));
                    break;
                case 2:
                    controllers.forEach(c -> c.setKD(c.getKD()+stepD));
                    break;
            }
        }
        if (dpadDown.onPress()) {
            switch (selectedPIDComponent.output()) {
                case 0:
                    controllers.forEach(c -> c.setKP(c.getKP()-stepP));
                    break;
                case 1:
                    controllers.forEach(c -> c.setKI(c.getKI()-stepI));
                    break;
                case 2:
                    controllers.forEach(c -> c.setKD(c.getKD()-stepD));
                    break;
            }
        }
        if (dpadLeft.onPress()) {
            switch (selectedPIDComponent.output()) {
                case 0:
                    stepP *= 10;
                    break;
                case 1:
                    stepI *= 10;
                    break;
                case 2:
                    stepD *= 10;
                    break;
            }
        }
        if (dpadRight.onPress()) {
            switch (selectedPIDComponent.output()) {
                case 0:
                    stepP /= 10;
                    break;
                case 1:
                    stepI /= 10;
                    break;
                case 2:
                    stepD /= 10;
                    break;
            }
        }

        if (btnA.onPress()) {
            if(controllers.size() > 0) {
                PIDController controller = controllers.get(0);

                switch(selectedPIDController.output()) {
                    case 0:
                        ReadWriteFile.writeFile(case0PIDTunedValuesFile,
                                String.format("%s|%s|%s",
                                        controller.getKP(),
                                        controller.getKI(),
                                        controller.getKD()));
                        break;
                    case 1:
                        ReadWriteFile.writeFile(case1PIDTunedValuesFile,
                                String.format("%s|%s|%s",
                                        controller.getKP(),
                                        controller.getKI(),
                                        controller.getKD()));
                        break;
                    case 2:
                        ReadWriteFile.writeFile(case2PIDTunedValuesFile,
                                String.format("%s|%s|%s",
                                        controller.getKP(),
                                        controller.getKI(),
                                        controller.getKD()));
                        break;
                }
            }

            telemetry.addData("Status", "Saved");
            telemetry.update();
        }

        if(bumperLeft.onPress()) {
            power -= 0.01;
        }

        if(bumperRight.onPress()) {
            power += 0.01;
        }
    }
}
