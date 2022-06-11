//
// Created by Trisoil on 2022/4/17.
//

#include "Navi_Control.h"
#include "Move_Controlor.h"
#include "Location_Module_Recieve.h"
#include "math.h"

Navi_PID_Number X_PID,Y_PID,Angle_PID;

void X_PID_Init(){
	X_PID.kp = 4329.0f;
	X_PID.ki = 0.004f;
	X_PID.kd = 6500.0f;
	X_PID.max_output  = 4000;
	X_PID.integral_limit = 10000;
	X_PID.deadbond = 0.01f;
	X_PID.max_err = 10;
}

void Y_PID_Init(){
	Y_PID.kp = 4329.0f;
	Y_PID.ki = 0.004f;
	Y_PID.kd = 6500.0f;
	Y_PID.max_output  = 4000;
	Y_PID.integral_limit = 10000;
	Y_PID.deadbond = 0.01f;
	Y_PID.max_err = 10;
}

void Angle_PID_Init(){
	Angle_PID.kp = 0.5f;
	Angle_PID.ki = 0;
	Angle_PID.kd = 0.4f;
	Angle_PID.max_output  = 100.0f;
	Angle_PID.integral_limit = 10000;
	Angle_PID.deadbond = 2.0f;
	Angle_PID.max_err = 10;
}

float Navi_PID_Calculate(float Set, Navi_PID_Number *PID_Control_Num){
	PID_Control_Num->ki_output += PID_Control_Num->ki * PID_Control_Num->now_err;
	if (PID_Control_Num->ki_output > PID_Control_Num->integral_limit) {
		PID_Control_Num->ki_output = PID_Control_Num->integral_limit;
	}else if (PID_Control_Num->ki_output < -PID_Control_Num->integral_limit){
		PID_Control_Num->ki_output = -PID_Control_Num->integral_limit;
	}
	PID_Control_Num->kp_output = PID_Control_Num->kp * PID_Control_Num->now_err;
	PID_Control_Num->kd_output = PID_Control_Num->kd * (PID_Control_Num->now_err - PID_Control_Num->last_err);
	PID_Control_Num->output = PID_Control_Num->ki_output + PID_Control_Num->kp_output + PID_Control_Num->kd_output;
	if (PID_Control_Num->output > PID_Control_Num->max_output){
		PID_Control_Num->output = PID_Control_Num->max_output;
	}else if (PID_Control_Num->output < -PID_Control_Num->max_output){
		PID_Control_Num->output = -PID_Control_Num->max_output;
	}
	return PID_Control_Num->output;
}

void Point_Navi_Move(float Set_X, float Set_Y, float Set_Angle)
{
	X_PID.last_err = X_PID.now_err;
	X_PID.now_err = Set_X - Location_Data.Pos_X;
	Y_PID.last_err = Y_PID.now_err;
	Y_PID.now_err = Set_Y - Location_Data.Pos_Y;
	Angle_PID.last_err = Angle_PID.now_err;
	Angle_PID.now_err = Set_Angle - Location_Data.Angle;
	float X_Speed = Navi_PID_Calculate(Set_X, &X_PID);
	float Y_Speed = Navi_PID_Calculate(Set_Y, &Y_PID);
	float Omega = Navi_PID_Calculate(Set_Angle, &Angle_PID);
	if (fabs((double)X_PID.now_err) < X_PID.deadbond)
	{
		X_Speed = 0;
	}
	if (fabs((double)Y_PID.now_err) < Y_PID.deadbond)
	{
		Y_Speed = 0;
	}
	if (fabs((double)Angle_PID.now_err) < Angle_PID.deadbond)
	{
		Omega = 0;
	}
	Gyro_Mode_Drive(X_Speed,-Y_Speed,Omega);
}

void Navi_Control_Init()
{
	X_PID_Init();
	Y_PID_Init();
	Angle_PID_Init();
}