package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBoard{

    Joystick BlueHalf;
    Joystick RedHalf;

    public ButtonBoard(Joystick Half1,Joystick Half2){
        RedHalf = Half2;
        BlueHalf = Half1;
    }

    public JoystickButton Blue1(){
        return new JoystickButton(BlueHalf,2);
    }
    public JoystickButton Blue2(){
        return new JoystickButton(BlueHalf,10);
    }
    public JoystickButton Blue3(){
        return new JoystickButton(BlueHalf,8);
    }
    public JoystickButton Blue4(){
        return new JoystickButton(BlueHalf,7);
    }
    public JoystickButton Blue5(){
        return new JoystickButton(BlueHalf,2);
    }
    public JoystickButton Blue6(){
        return new JoystickButton(BlueHalf,9);
    }
    public JoystickButton Blue7(){
        return new JoystickButton(BlueHalf,6);
    }
    public JoystickButton Blue8(){
        return new JoystickButton(BlueHalf,5);
    }
    public JoystickButton Blue9(){
        return new JoystickButton(BlueHalf,4);
    }
    public JoystickButton Blue10(){
        return new JoystickButton(BlueHalf,3);
    }
    public JoystickButton Red1(){
        return new JoystickButton(BlueHalf,11);
    }
    public JoystickButton Red2(){
        return new JoystickButton(RedHalf,11);
    }
    public JoystickButton Red3(){
        return new JoystickButton(RedHalf,10);
    }
    public JoystickButton Red4(){
        return new JoystickButton(RedHalf,9);
    }
    public JoystickButton Red5(){
        return new JoystickButton(RedHalf,8);
    }
    public JoystickButton Red6(){
        return new JoystickButton(RedHalf,7);
    }
    public JoystickButton Red7(){
        return new JoystickButton(RedHalf,4);
    }
    public JoystickButton Red8(){
        return new JoystickButton(RedHalf,5);
    }
    public JoystickButton Red9(){
        return new JoystickButton(RedHalf,6);
    }
    public JoystickButton Red10(){
        return new JoystickButton(RedHalf,12);
    }

    

}