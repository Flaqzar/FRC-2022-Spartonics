package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.math.Vec2d;

public class ControllerHandler
{
	public static boolean getButton(EnumButton button, XboxController... controllers)
	{
		for(int i = 0; i < controllers.length; i++)
		{
			if(controllers[i].getRawButton(button.getId()))
			{
				return true;
			}
		}

		return false;
	}

	public static double getAxis(EnumAxis axis, XboxController... controllers)
	{
		double value = 0d;

		for(int i = 0; i < controllers.length; i++)
		{
			if(Math.abs(controllers[i].getRawAxis(0)) > Math.abs(value))
			{
				value = controllers[i].getRawAxis(axis.getId());
			}
		}

		return value;
	}

	public static int getDPad(XboxController... controllers)
	{
		for(int i = 0; i < controllers.length; i++)
		{
			if(controllers[i].getPOV() != -1)
			{
				return controllers[i].getPOV();
			}
		}

		return -1;
	}

	public static Vec2d getJoystick(EnumJoystick joystick, XboxController... controllers)
	{
		Vec2d[] values = new Vec2d[controllers.length];

		for(int i = 0; i < controllers.length; i++)
		{
			switch(joystick)
			{
			case LEFT_JOYSTICK:
				values[i] = new Vec2d(controllers[i].getRawAxis(EnumAxis.LEFT_JOYSTICK_X.getId()), controllers[i].getRawAxis(-EnumAxis.LEFT_JOYSTICK_Y.getId()));
				break;
			case RIGHT_JOYSTICK:
				values[i] = new Vec2d(controllers[i].getRawAxis(EnumAxis.RIGHT_JOYSTICK_X.getId()), controllers[i].getRawAxis(-EnumAxis.RIGHT_JOYSTICK_Y.getId()));
				break;
			case DPAD:
				values[i] = controllers[0].getPOV() == -1 ? new Vec2d(0d, 0d) : new Vec2d(controllers[0].getPOV(), true);
				break;
			}
		}
		
		Arrays.sort(values, (a, b) -> Double.compare(a.getLengthSquared(), b.getLengthSquared()));

		return values[0];
	}

	public static enum EnumButton
	{
		A_BUTTON(0),
		B_BUTTON(1),
		X_BUTTON(2),
		Y_BUTTON(3),
		LEFT_BUMPER(4),
		RIGHT_BUMPER(5),
		MINUS(6),
		PLUS(7),
		LEFT_JOYSTICK(8),
		RIGHT_JOYSTICK(9);

		private final int id;

		private EnumButton(int buttonId)
		{
			this.id = buttonId;
		}

		public int getId()
		{
			return this.id;
		}
	}

	public static enum EnumAxis
	{
		LEFT_JOYSTICK_X(0),
		LEFT_JOYSTICK_Y(1),
		LEFT_TRIGGER(2),
		RIGHT_TRIGGER(3),
		RIGHT_JOYSTICK_X(4),
		RIGHT_JOYSTICK_Y(5);

		private final int id;

		private EnumAxis(int buttonId)
		{
			this.id = buttonId;
		}

		public int getId()
		{
			return this.id;
		}
	}

	public static enum EnumJoystick
	{
		LEFT_JOYSTICK,
		RIGHT_JOYSTICK,
		DPAD;
	}
}
