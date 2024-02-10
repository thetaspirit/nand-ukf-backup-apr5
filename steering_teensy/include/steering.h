#pragma once

// Steering is left-positive and measured in degrees from straight ahead.
namespace steering {
	void init();

	// Determine the left and right angle limits and then center the wheel.
	// This will block until the calibration is done.
	void calibrate();

	// Set the position (in degrees from center, left-positive) that the stepper will try to achieve.
	void set_goal_angle(float degrees);

	// Returns true if the alarm pin has been triggered,
	// meaning that the stepper driver is no longer responding and we have lost steering control.
	// A full power cycle of both the Teensy and driver is required to clear this condition.
	bool alarm_triggered();

	int left_step_limit();
	int right_step_limit();
}