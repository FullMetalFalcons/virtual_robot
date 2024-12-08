package virtual_robot.robots.classes;

import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;

/**
 * For internal use only. Represents a robot with four mecanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and an extensible arm with a grabber on the end.
 *
 * ArmBot extends the MecanumPhysicsBase class, which manages the drive train and various sensors, as well as
 * basic interaction between the chassis and the walls and game elements.
 *
 * ArmBot adds the extensible arm, including both the graphical display and the dyn4j Bodys that represent the
 * arm. Any "special" interaction between the robot and game elements must be handled by ArmBot.
 *
 * The @BotConfig annotation is required. The name will be displayed to the user in the Configuration
 * combo box. The filename refers to the fxml file that contains the markup for the graphical UI.
 * Note: the fxml file must be located in the virtual_robot.robots.classes.fxml folder.
 */
//@BotConfig(name = "Arm Bot Penguins", filename = "arm_bot_penguins")
public class ArmBotPenguinsProfileView { //extends MecanumPhysicsBase {

    final double INITIAL_ARM_WIDTH = 75;

    /*
    Variables representing graphical UI nodes that we will need to manipulate. The @FXML annotation will
    cause these variables to be instantiated automatically during loading of the arm_bot.fxml file. The
    fxml file must declare fx:id attributes for the Rectangles that represent the arm, hand, and both fingers.
    For example, the attribute for the arm would be:  fx:id="arm"
     */
    @FXML private Group armProfileGroup;
    @FXML private Rectangle armProfileArm;

    /*
     * Transform objects that will be instantiated in the initialize() method, and will be used in the
     * updateDisplay() method to manipulate the arm, hand, and fingers.
     */
    private Translate profileArmTranslate;
    private Rotate profileArmRotate;

    private double initialArmWidthPx;
    private double initialArmHeightPx;

    /**
     * Constructor.
     */
    public ArmBotPenguinsProfileView() {
    }

    /**
     *  The initialize() method is called automatically when the robot's graphical UI is loaded from the
     *  arm_bot.fxml markup file. It should be used to set up parts of the graphical UI that will change
     *  as the robot operates
     */
    public void initialize(){
        initialArmWidthPx = armProfileArm.getWidth();
        initialArmHeightPx = armProfileArm.getHeight();

        // This will be used to extend the arm
        profileArmTranslate = new Translate(0, 0);
        armProfileGroup.getTransforms().add(profileArmTranslate);

        // This will be used to display the arm at the correct angle
        // The initial pivot point is the top left of the mini-robot so move
        //    the pivot down to match how far down the arm was originally placed
        //    minus half the height
        profileArmRotate = new Rotate(0, 0, armProfileArm.getY()-(initialArmHeightPx /2));
        armProfileGroup.getTransforms().add(profileArmRotate);

    }
    /**
     *  Update the display of the robot UI. This method will be called from the UI Thread via a call to
     *  Platform.runLater().
     */
    public synchronized void updateDisplay(double armAddtlLengthPixels, double armAngleDegrees){
        //Negative degrees moves the right edge of the arm up
        profileArmRotate.setAngle(-armAngleDegrees);

        //To extend, we need to make the arm longer, but we also have to shift the
        //  arm over so the pivot stays at the same spot on the left edge
        armProfileArm.setWidth(initialArmWidthPx + armAddtlLengthPixels);
    }



}
