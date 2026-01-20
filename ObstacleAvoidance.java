package exercises.e2;

import robot.Robot;
import utils.Delay;

/**
 * Created by Theo Theodoridis.
 * Class    : ObstacleAvoidance
 * Version  : v1.0
 * Date     : © Copyright 2022
 * User     : ttheod
 * email    : t.theodoridis@salford.ac.uk
 * Comments : Implementation of an obstacle avoidance behaviour using min vector ranges.
 **/

public class ObstacleAvoidance
{
    // [+]Avoidance declarations:
    private static final double GAMMA_A  = 250.0;

    // [+]Untrap declarations:
    int l_cnt = 0;
    int r_cnt = 0;
    boolean l_trap = false;
    boolean r_trap = false;
    private double vel = 0.0;

    private Robot robot;

   /**
    * Method     : ObstacleAvoidance::ObstacleAvoidance()
    * Purpose    : Secondary ObstacleAvoidance constructor.
    * Parameters : - robot : An object of the Robot class.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public ObstacleAvoidance(Robot robot)
    {
        this.robot = robot;
    }

   /**
    * Method     : ObstacleAvoidance::decollide()
    * Purpose    : Perform collision detection and avoidance.
    * Parameters : - vel : The robot velocity.
    * Returns    : True if collision is detected, False otherwise.
    * Notes      : Makes use of the two robot velocities (feedback).
    **/
    public boolean decollide(double vel)
    {
        // [1]Get initial pose and zero velocity flags:
        boolean initPose = (robot.kinematics.getX() == 0) && (robot.kinematics.getY() == 0);
        boolean zeroVel  = (robot.kinematics.getLeftVel() == 0) && (robot.kinematics.getRightVel() == 0);

        // [2]Validate flags and perform a back and random
        // turn motions:
        if(zeroVel && !initPose)
        {
            robot.control.move(-vel); // Move backward.
            Delay.ms(2000);

            double p = Math.random();
            if(p < 0.5) robot.control.turnSpot(vel);
            else        robot.control.turnSpot(-vel);
            Delay.ms(2000);
            return(true);
        }
        else
        // [3]Otherwise, move forward:
        {
            robot.control.move(vel); // Move forward.
        }
        return(false);
    }

   /**
    * Method     : ObstacleAvoidance::avoid()
    * Purpose    : Perform obstacle detection and avoidance.
    * Parameters : - vel : The robot velocity.
    * Returns    : True if obstacle is detected, False otherwise.
    * Notes      : Makes use of the 6 ultrasonic sensors.
    **/
    public boolean avoid(double vel)
    {
        // [1]Collect left/right sensor ranges:
        double l_vec[] = {
            robot.sensor.getSonarRange(1),
            robot.sensor.getSonarRange(2),
            robot.sensor.getSonarRange(3)
        };
        double r_vec[] = {
            robot.sensor.getSonarRange(6),
            robot.sensor.getSonarRange(7),
            robot.sensor.getSonarRange(8)
        };

        // [2]Calculate the left/right min sensor vector:
        double l_min = Math.min(Math.min(l_vec[0], l_vec[1]), l_vec[2]);
        double r_min = Math.min(Math.min(r_vec[0], r_vec[1]), r_vec[2]);

        // [3]Validate if an obstacle is detected left and turn right:
        if(l_min < GAMMA_A)
        {
            this.vel = +vel;
            robot.control.turnSpot(this.vel);
            return(true);
        }
        else
        // [4]Validate if an obstacle is detected right and turn left:
        if(r_min < GAMMA_A)
        {
            this.vel = vel;
            robot.control.turnSpot(this.vel);
            return(true);
        }
        else
        // [5]Otherwise, invoke decollider:
        {
            // Decollision recursive method call:
            decollide(vel);
        }

        return(false);
    }

   /**
    * Method     : ObstacleAvoidance::untrap()
    * Purpose    : Perform corner trap detection and avoidance.
    * Parameters : - vel : The robot velocity.
    * Returns    : True if a trap is detected, False otherwise.
    * Notes      : Makes use of the 6 ultrasonic sensors.
    **/
    public boolean untrap(double vel)
    {
        // [1]Detection phase:
        if((this.vel >= 0) && !l_trap)
        {
            l_cnt++;
            l_trap = true;
            r_trap = false;
        }
        else
        if((this.vel < 0) && !r_trap)
        {
            r_cnt++;
            r_trap = true;
            l_trap = false;
        }

        // [2]Escaping phase:
        if((l_cnt >= 2) && (r_cnt <= 1))
        {
            //System.out.println("L-Trap");
            robot.control.turnSpot(+vel);
            Delay.ms(2000);
            l_cnt = 0;
            r_cnt = 0;
            l_trap = false;
            r_trap = false;
            return(true);
        }
        else
        if((r_cnt >= 2) && (l_cnt <= 1))
        {
            //System.out.println("R-Trap");
            robot.control.turnSpot(-vel);
            Delay.ms(2000);
            l_cnt = 0;
            r_cnt = 0;
            l_trap = false;
            r_trap = false;
            return(true);
        }
        else
        {
            // Avoidance recursive method call:
            avoid(vel);
        }
        return(false);
    }
}
