/*
 * RmdMotorCAN.cpp
 *
 *  Created on: Dec 7, 2024
 *      Author: Yannick
 */


#include "target_constants.h"
#ifdef RMDCAN
#include "RmdMotorCAN.h"

bool RmdMotorCAN1::inUse = false;
ClassIdentifier RmdMotorCAN1::info = {
		 .name = "RMD MyActuator (1)" ,
		 .id=CLSID_MOT_RMD1,	// 11
};
bool RmdMotorCAN2::inUse = false;
ClassIdentifier RmdMotorCAN2::info = {
		 .name = "RMD MyActuator (2)" ,
		 .id=CLSID_MOT_RMD2,	// 12
};

const ClassIdentifier RmdMotorCAN1::getInfo(){
	return info;
}

const ClassIdentifier RmdMotorCAN2::getInfo(){
	return info;
}

bool RmdMotorCAN1::isCreatable(){
	return !RmdMotorCAN1::inUse; // Creatable if not already in use for example by another axis
}

bool RmdMotorCAN2::isCreatable(){
	return !RmdMotorCAN2::inUse; // Creatable if not already in use for example by another axis
}



RmdMotorCAN::RmdMotorCAN(uint8_t instance) : CommandHandler("rmd", CLSID_MOT_RMD1,instance),  Thread("RMD", RMD_THREAD_MEM, RMD_THREAD_PRIO), motorId(instance) {
	if(motorId == 0){
		nodeId = 1;
	}else if(motorId == 1){
		nodeId = 2; // defaults
	}
	restoreFlash();
	setCanFilter();

	if(port->getSpeedPreset() < 3){
		port->setSpeedPreset(3); // Minimum 250k. Default 1M
	}
	this->robstriteMotor = RobStrite_Motor([this](void *header, uint8_t *data){sendMsg(header,data);},
			[this](uint32_t delay){Delay(delay);});
	this->robstriteMotor.SetSenderCanID(nodeId); // Set CAN ID for robstrite motor
	this->port->setSilentMode(false);
	this->registerCommands();
	this->port->takePort();
	this->Start();
}

RmdMotorCAN::~RmdMotorCAN() {
	this->port->removeCanFilter(filterId);
	stopMotor();
}


void RmdMotorCAN::restoreFlash(){
	uint16_t data = 0;
	if(Flash_Read(motorId == 0 ? ADR_RMD1_DATA1 : ADR_RMD2_DATA1, &data)){
		this->nodeId = (data & 0x1F) + 1; // Valid ID 1-32
		// Bits 7-15 free
		this->activerequests = (data >> 6) & 0x1;
	}
	if(Flash_Read(motorId == 0 ? ADR_RMD1_TORQUE : ADR_RMD2_TORQUE, &data)){
		maxTorque = data & 0x7FFF;
		// Bit 15 free
	}

	// Offset. Could be stored in motor but does not work reliably
	if(Flash_Read(motorId == 0 ? ADR_RMD1_OFFSET : ADR_RMD2_OFFSET, &data)){
		posOffset = (int16_t)(data);
	}

}
void RmdMotorCAN::saveFlash(){
	// Save CAN ID and max torque
	uint16_t data1 = (this->nodeId-1) & 0x1F;
	data1 |= this->activerequests ? 1 << 6 : 0;
	Flash_Write(motorId == 0 ? ADR_RMD1_DATA1 : ADR_RMD2_DATA1,data1);
	uint16_t torquedat = maxTorque & 0x7FFF;
	Flash_Write(motorId == 0 ? ADR_RMD1_TORQUE : ADR_RMD2_TORQUE, torquedat);

	int32_t offset = (posOffset % (posOffset >= 0 ? 36000 : -36000));

	Flash_Write(motorId == 0 ? ADR_RMD1_OFFSET : ADR_RMD2_OFFSET, (uint16_t)offset);
}

void RmdMotorCAN::Run(){
//	bool first = true;
	Delay(100);
	while(true){
        auto current_time = HAL_GetTick();
        if(activerequests)
		{
            robstriteMotor.Enable_Motor(); // 获得反馈
        }
		available=true;
		if(current_time-_lastVoltageUpdateTime > 2000)
		{
			_lastVoltageUpdateTime = current_time;
			robstriteMotor.Get_RobStrite_Motor_parameter(0x701C); //获得电压
		}
		DelayUntil(500);
		
		// nextAvailable = false;
		// if(!available){
		// 	// sendCmd(0xB5); // Get type
		// 	Delay(20);
		// }

		// if(activerequests){
		// 	updateStatus();
		// }else{
		// 	if(!available || requestConstantReportEnable){
		// 		// Setup
		// 		requestConstantReports(0x92, true, 0); // Enable constant position sending
		// 		requestConstantReportEnable = false;
		// 		Delay(150);
		// 	}
		// }
		// Delay(1000);
		// available = nextAvailable; // should have received some replies
	}
}

void RmdMotorCAN::setCanFilter(){
	if(filterId >= 0){
		port->removeCanFilter(filterId);
	}
	// nodeId = std::min<uint8_t>(nodeId,32);
	// uint32_t filter_id = (nodeId + 0x240); // Reply
	// uint32_t filter_mask = 0x3FF;

	// CAN_filter filterConf;
	// filterConf.buffer = motorId % 2 == 0 ? 0 : 1;
	// filterConf.filter_id = filter_id;
	// filterConf.filter_mask =  filter_mask;
	// this->filterId = this->port->addCanFilter(filterConf);
	//改成全接受
    CAN_filter filterConf;
    filterConf.buffer = 0;               // 默认缓冲区，你可以根据需要调整
    filterConf.filter_id = 0x00000000;   // 匹配所有 ID
    filterConf.filter_mask = 0x00000000; // 屏蔽所有位（即所有 ID 都能匹配）

    this->filterId = this->port->addCanFilter(filterConf);
}

void RmdMotorCAN::stopMotor(){
	active = false;
	// sendCmd(0x80); // Disable motor
	robstriteMotor.Disenable_Motor(1);
}

void RmdMotorCAN::startMotor(){
	if(lastErrors.asInt == 0){ // Only allow enabling if no errors to prevent reenabling after failure
		active = true;
		// setTorque(0); // Enable torque mode, no torque
		robstriteMotor.Enable_Motor();
	}
}

Encoder* RmdMotorCAN::getEncoder(){
	return static_cast<Encoder*>(this);
}

bool RmdMotorCAN::motorReady(){

	return lastErrors.asInt == 0 && available; // Ping motor state
}


uint32_t RmdMotorCAN::getCpr(){
	return 36000;
}

int32_t RmdMotorCAN::getPos(){
	if(activerequests && HAL_GetTick() - lastAngleUpdate > angleUpdateMs){
		// pos outdated. Should be sent without request
		robstriteMotor.Enable_Motor();
	}
	lastPos= robstriteMotor.Pos_Info.Angle; // Get position in 0.01 degrees
	return (lastPos*100) - posOffset;
}

void RmdMotorCAN::setPos(int32_t pos){
	posOffset = (lastPos*100) - pos;
}
void RmdMotorCAN::sendMsg(void* header_in,uint8_t* data){
	CAN_TxHeaderTypeDef * header= static_cast<CAN_TxHeaderTypeDef *>(header_in); 
  	CAN_tx_msg msg;
	//用的是extid 在这里extid是标志位决定id被当成stdid还是extid
	msg.header.extId= true;
	msg.header.id = header->ExtId;
	msg.header.rtr = header->RTR;
	msg.header.length = header->DLC;
	memcpy(&msg.data,data,header->DLC);
	if(!port->sendMessage(msg)){
		// Nothing
	}
}
void RmdMotorCAN::sendMsg(std::array<uint8_t,8> &data,uint8_t len){
	CAN_tx_msg msg;
	memcpy(&msg.data,data.data(),std::min<uint8_t>(data.size(), CAN_MSGBUFSIZE));
	msg.header.id = this->nodeId + 0x140;
	msg.header.length = len;
	if(!port->sendMessage(msg)){
		// Nothing
	}
}


/**
 * Torque 0.01A * torque
 */
void RmdMotorCAN::setTorque(int16_t torque){
	std::array<uint8_t,8> data{0xA1,0,0,0,(uint8_t)(torque & 0xff),(uint8_t)((torque >> 8) & 0xff),0,0};
	sendMsg(data);
}

void RmdMotorCAN::turn(int16_t power){
	if(!active || !available){
		return;
	}
	int16_t torque = ((float)power / (float)0x7fff) * maxTorque;
	setTorque(torque);
}


EncoderType RmdMotorCAN::getEncoderType(){
	return EncoderType::absolute;
}

void RmdMotorCAN::errorCb(ErrorStatus &errors){
	if(errors.asInt){
		stopMotor();
	}
}


void RmdMotorCAN::canRxPendCallback(CANPort* port,CAN_rx_msg& msg){
	if(msg.header.extId==false)
	{
		return; // Ignore standard ID
	}
	uint32_t extid= msg.header.id;
	robstriteMotor.RobStrite_Motor_Analysis(msg.data, extid);
	curTemp=robstriteMotor.Pos_Info.Temp;
	curVoltage=robstriteMotor.drw.VBUS.data*10;
}

void RmdMotorCAN::registerCommands(){
	CommandHandler::registerCommands();
	registerCommand("canid", RmdCAN_commands::canid, "CAN id of motor",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("errors", RmdCAN_commands::errors, "Error flags",CMDFLAG_GET);
	registerCommand("maxtorque", RmdCAN_commands::maxtorque, "Maximum motor current in 0.01A (When activerequests on)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("curr", RmdCAN_commands::current, "Current in 0.01A (When activerequests on)",CMDFLAG_GET);
	registerCommand("temp", RmdCAN_commands::temperature, "Temperature in °C (When activerequests on)",CMDFLAG_GET);
	registerCommand("vbus", RmdCAN_commands::voltage, "Voltage in 0.1V (When activerequests on)",CMDFLAG_GET);
	registerCommand("requestpos", RmdCAN_commands::activerequests, "1 to send active position requests for higher rates",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("model", RmdCAN_commands::modelname, "Name of motor",CMDFLAG_GET | CMDFLAG_STR_ONLY);
}

CommandStatus RmdMotorCAN::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){

	switch(static_cast<RmdCAN_commands>(cmd.cmdId)){
	case RmdCAN_commands::canid:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->nodeId);
		}else if(cmd.type == CMDtype::set){
			if(cmd.val != this->nodeId){
				this->nodeId = cmd.val;
				setCanFilter(); // Removes previous filter if set automatically
				robstriteMotor.SetSenderCanID(this->nodeId);
			}
		}else{
			return CommandStatus::ERR;
		}
		break;

	case RmdCAN_commands::maxtorque:
			handleGetSet(cmd, replies, this->maxTorque);
			break;
	case RmdCAN_commands::current:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(curCurrent);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case RmdCAN_commands::temperature:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(curTemp);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case RmdCAN_commands::errors:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(lastErrors.asInt);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case RmdCAN_commands::voltage:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(curVoltage);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case RmdCAN_commands::modelname:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->modelName);
		}else{
			return CommandStatus::ERR;
		}
			break;

	case RmdCAN_commands::activerequests:
		if(cmd.type == CMDtype::get){
				replies.emplace_back(activerequests);
			}else if(cmd.type == CMDtype::set){
				// updateRequestMode(cmd.val);
				this->activerequests=cmd.val;
			}else{
				return CommandStatus::ERR;
			}
		break;

	default:
		return CommandStatus::NOT_FOUND;
	}

	return CommandStatus::OK;

}


#endif
