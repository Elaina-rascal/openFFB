/*
 * RmdMotorCAN.h
 *
 *  Created on: Dec 7, 2024
 *      Author: Yannick
 */

#ifndef USEREXTENSIONS_SRC_RMDMOTORCAN_H_
#define USEREXTENSIONS_SRC_RMDMOTORCAN_H_

#include "MotorDriver.h"
#include "cpp_target_config.h"
#include "CAN.h"
#include "Encoder.h"
#include "thread.hpp"
#include "CanHandler.h"
#include "CommandHandler.h"
#include "PersistentStorage.h"
#include "Robstrite.h"
#ifdef RMDCAN
#define RMD_THREAD_MEM 256
#define RMD_THREAD_PRIO 25 // Must be higher than main thread

enum class RmdCAN_commands : uint32_t{
	canid,errors,maxtorque,current,temperature,voltage,activerequests,modelname
};

class RmdMotorCAN : public MotorDriver,public PersistentStorage, public Encoder, public CanHandler, public CommandHandler, cpp_freertos::Thread {
public:
	RmdMotorCAN(uint8_t instance);
	virtual ~RmdMotorCAN();

	const ClassIdentifier getInfo() = 0;
	void turn(int16_t power) override;
	void stopMotor() override;
	void startMotor() override;
	Encoder* getEncoder() override;
	bool hasIntegratedEncoder() override {return true;}
	void setTorque(float torque);
	bool motorReady() override; 

	void Run()override;

	void saveFlash() override; 		// Write to flash here
	void restoreFlash() override;	// Load from flash

//	float getPos_f() override;
	uint32_t getCpr() override;
	int32_t getPos() override;
	float getPos_f() override;
	void setPos(int32_t pos) override;
	EncoderType getEncoderType() override;

	void setCanFilter();
	void sendMsg(void* header,uint8_t* data);
	void sendMsg(std::array<uint8_t,8> &data,uint8_t len = 8);

	void canRxPendCallback(CANPort* port,CAN_rx_msg& msg) override;

	void registerCommands();
	CommandStatus command(const ParsedCommand& cmd,std::vector<CommandReply>& replies) override;

	const uint32_t angleUpdateMs = 10;

	union ErrorStatus {
		struct ErrorStatur_s{
			uint16_t res1 : 1,
			stall : 1,
			undervoltage : 1,
			overvoltage : 1,
			overcurrent : 1,
			overpower : 1,
			calibrationWriteErr : 1,
			overspeed : 1,
			overtemp : 1,
			calibrationErr : 1,
			res2 : 6;
		};
		uint16_t asInt;
		ErrorStatur_s flags;
	};
private:
	CANPort* port = &canport;
	uint8_t nodeId = 1; // can ID 1-32
	uint8_t motorId = 0;
	bool active = false;
	bool nextAvailable = false;
	bool available = false;
	bool activerequests = false;
	bool requestConstantReportEnable = false;

	char modelName[8] = {"strite"};

	uint32_t lastAngleUpdate = 0;
	uint32_t lastTorqueStatusUpdate_us = 0;
	uint32_t _lastVoltageUpdateTime = 0;
	float lastPos = 0;
	uint16_t maxTorque = 1000; // in 0.01A, max 0x7fff

	int32_t filterId = -2;

	int16_t curCurrent = 0;
	float curTemp = 0;
	float curVoltage = 0;

	ErrorStatus lastErrors = {0};

	int32_t posOffset = 0;
	RobStrite_Motor robstriteMotor;
	void errorCb(ErrorStatus &errors);
};


/**
 * Instance 1 of RMD motor
 */
class RmdMotorCAN1 : public RmdMotorCAN{
public:
	RmdMotorCAN1() : RmdMotorCAN{0} {inUse = true;}
	const ClassIdentifier getInfo();
	~RmdMotorCAN1(){inUse = false;}
	static bool isCreatable();
	static ClassIdentifier info;
	static bool inUse;
};

/**
 * Instance 2 of RMD motor
 */
class RmdMotorCAN2 : public RmdMotorCAN{
public:
	RmdMotorCAN2() : RmdMotorCAN{1} {inUse = true;}
	const ClassIdentifier getInfo();
	~RmdMotorCAN2(){inUse = false;}
	static bool isCreatable();
	static ClassIdentifier info;
	static bool inUse;
};
#endif
#endif /* USEREXTENSIONS_SRC_RMDMOTORCAN_H_ */
