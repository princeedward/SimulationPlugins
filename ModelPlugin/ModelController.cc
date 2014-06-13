#include "ModelController.hh"

using std::vector;
using std::string;
using std::cout;
using std::endl;

namespace gazebo{
ModelController::ModelController() 
		: ModelPlugin(), JointAngleKPID(5.5,0,0.75), ModelAngleKPID(1,0,0)
{
	// Initialize variables
	WheelRadius =  0.045275;
	MaxiRotationRate = 2.4086;
	AccelerationRate = 8;
	PlanarMotionStopThreshold = 0.016;

	executionSate = 0;
	startExecution = false;
	commandPriority = 0;

	// A hint of model been initialized
	printf("Model Initiated\n");
}
ModelController::~ModelController()
{
	this->model.reset();
	this->jointWR.reset();
	this->jointWL.reset();
	this->jointWF.reset();
	this->jointCB.reset();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be executed When Model Has Been Loaded
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	// Initialize the whole system
	SystemInitialization(_parent);
	// Initialize the subscribers
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	this->welcomeInfoSub = node->Subscribe("~/Welcome",&ModelController::WelcomInfoProcessor, this);
	// commandSubscriber = node->Subscribe("~/collision_map/command", &CollisionMapCreator::create, this);
	
	// Bind function which will be executed in each iteration
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelController::OnSystemRunning, this, _1));
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be executed When model received welcome message from world plugin
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::WelcomInfoProcessor(GzStringPtr &msg)
{
	string InfoReceived = msg->data();
	// Do something useful here
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be called in function Load() to initialize the model
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::SystemInitialization(physics::ModelPtr parentModel)
{
	// Get all the pointers point to right objects
	this->model = parentModel;
	this->jointWR = model->GetJoint("Right_wheel_hinge");
	this->jointWL = model->GetJoint("Left_wheel_hinge");
	this->jointWF = model->GetJoint("Front_wheel_hinge");
	this->jointCB = model->GetJoint("Center_hinge");
	jointWRP.JointX = jointWR;
	jointWRP.Need2BeSet = false;
	jointWRP.JointErrorHis = 0;
	jointWRP.JointErrorAccu = 0;
	jointWLP.JointX = jointWL;
	jointWLP.Need2BeSet = false;
	jointWLP.JointErrorHis = 0;
	jointWLP.JointErrorAccu = 0;
	jointWFP.JointX = jointWF;
	jointWFP.Need2BeSet = false;
	jointWFP.JointErrorHis = 0;
	jointWFP.JointErrorAccu = 0;
	jointCBP.JointX = jointCB;
	jointCBP.Need2BeSet = false;
	jointCBP.JointErrorHis = 0;
	jointCBP.JointErrorAccu = 0;
	// Setting the model states
	// Setting the maximium torque of the two wheels
	this->jointWR->SetMaxForce(0,jointWR->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->Get<double>());
	this->jointWL->SetMaxForce(0,jointWL->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->Get<double>());
	// Setting the maximium torque of the front wheel
	this->jointWF->SetMaxForce(0,jointWF->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->Get<double>());
	// Setting the maximium torque of the body bending joint
	this->jointCB->SetMaxForce(0,jointCB->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->Get<double>());
	// Set the angle of the hinge in the center to zero
	// math::Angle InitialAngle(0.00);
	// this->jointCB->SetAngle(0, InitialAngle);
	// math::Angle AngleNeed2Be(0.49778);
	// this->jointCB->SetAngle(0, AngleNeed2Be);

	
	// physics::ModelState CurrentModelState(model);
	string TopicName = "~/" + model->GetName() + "_world";
	string TopicNamePub = "~/" + model->GetName() + "_model";
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init(model->GetName());
	this->commandSub = node->Subscribe(TopicName,&ModelController::CommandDecoding, this);
	this->commandPub = node->Advertise<command_message::msgs::CommandMessage>(TopicNamePub);
	CollisionPubAndSubInitialization();
	
	command_message::msgs::CommandMessage feed_back_message;
	feed_back_message.set_messagetype(5);
	feed_back_message.set_stringmessage(model->GetName());
	commandPub->Publish(feed_back_message);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be called in each iteration of simulation
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::OnSystemRunning(const common::UpdateInfo & /*_info*/)
{
	// Update variables that need to be updated in each iteration
	JointAngleUpdateInJointPlus();
	// Do something useful after simulation begins
	if (executionSate == 1)
	{
		math::Vector2d final_position(targetPosition.pos.x,targetPosition.pos.y);
		math::Angle final_orientation(targetPosition.pos.z);
		Move2Point(final_position,final_orientation);
		if (startExecution)
		{
			PositionTracking();
		}
	}
	if (executionSate == 2)
	{
		for (int i = 0; i < 4; ++i)
		{
			JointPIDController(GetJointPlus(i), jointAngleShouldBe[i]);
		}
		if (startExecution)
		{
			JointAngleTracking();
		}
	}
	if (executionSate == 3)
	{
		JointPIDController(GetJointPlus(0), jointAngleShouldBe[0]);
		JointPIDController(GetJointPlus(3), jointAngleShouldBe[3]);
		SetJointSpeed(jointWL, 0, lftWheelSpeed);
		SetJointSpeed(jointWR, 0, rgtWheelSpeed);
	}
}

//-------------------------------------------------------------------
// Functions used below are for magnetic connection simulation
//-------------------------------------------------------------------

//------------------- Start Region ----------------------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is the initalization function of collision message
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::CollisionPubAndSubInitialization(void)
{
	gazebo::transport::NodePtr node1(new gazebo::transport::Node());
	// Initialize the node with the model name
	node1->Init(model->GetName());
	// cout<<"Mode: node name is '"<<model->GetName()<<"'"<<endl;
	string TopicName = "~/" + model->GetName() + "::FrontWheel::front_contact";
	// cout<<"Mode: node topic is '"<<TopicName<<"'"<<endl;
	this->linkCollisonSub[0] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);
	TopicName = "~/" + model->GetName() + "::UHolderBody::UHolder_contact";
	// cout<<"Mode: node topic is '"<<TopicName<<"'"<<endl;
	this->linkCollisonSub[1] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);
	TopicName = "~/" + model->GetName() + "::LeftWheel::LeftWheel_contact";
	this->linkCollisonSub[2] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);
	TopicName = "~/" + model->GetName() + "::RightWheel::RightWheel_contact";
	this->linkCollisonSub[3] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);

	string ColPubName = "~/"+model->GetName()+"_Collision";
	collisionInfoToServer = node1->Advertise<collision_message_plus::msgs::CollisionMessage>(ColPubName);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be called when recieved collision message
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::CollisionReceiverProcessor(GzStringPtr &msg)
{
	// cout<<"Message: "<<msg->data()<<endl;
	string MsgsInfo = msg->data();
	string Collison1 = MsgsInfo.substr(0,MsgsInfo.find(","));
	string Collison2 = MsgsInfo.substr(MsgsInfo.find(",")+1,-1);
	// cout<<"Model: Collision 1: "<<Collison1<<endl;
	// cout<<"Model: Collision 2: "<<Collison2<<endl;
	collision_message_plus::msgs::CollisionMessage CollisionMsgsPush;

	string LinkName;
	// cout<<"Model: Information came from model: "<<model->GetName()<<endl;
	if(Collison1.substr(0,Collison1.find("::")).compare(model->GetName())==0)
	{
		// cout<<"Model: Collision 1 position "
		CollisionMsgsPush.set_collision1(Collison1);
		CollisionMsgsPush.set_collision2(Collison2);
		LinkName = Collison1.substr((model->GetName()).length()+2,Collison1.find(":",(model->GetName()).length()+2)-2-(model->GetName()).length());
	}else{
		CollisionMsgsPush.set_collision1(Collison2);
		CollisionMsgsPush.set_collision2(Collison1);
		LinkName = Collison2.substr((model->GetName()).length()+2,Collison2.find(":",(model->GetName()).length()+2)-2-(model->GetName()).length());
	}
	// cout<<"Model: The collision link name is :"<<LinkName<<endl;
	msgs::Pose PositionOfCollisionLink = msgs::Convert(model->GetLink(LinkName)->GetWorldPose());
	CollisionMsgsPush.mutable_positioncol1()->CopyFrom(PositionOfCollisionLink);

	collisionInfoToServer->Publish(CollisionMsgsPush);
}

//------------------- End Region ----------------------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be called in the OnSystemRunning(), once for a control period
//            The format of the command might be a gait table
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::CommandDecoding(CommandMessagePtr &msg)
{
	int commandType = msg->messagetype();
	command_message::msgs::CommandMessage feed_back_message;
	feed_back_message.set_messagetype(0);
	feed_back_message.set_stringmessage(model->GetName()+":");
	switch(commandType)
	{
		// This line may need to be changed
		// When using the new representation, the old connection management mechanism may be unnecessary
		case 1:{nameOfConnectedModels.push_back(msg->stringmessage());break;}
		case 2:
		{
			this->executionSate = 1;
			this->targetPosition = gazebo::msgs::Convert(msg->positionneedtobe());
			commandPub->Publish(feed_back_message);
			break;
		}
		case 3:
		{
			this->executionSate = 2;
			for (int i = 0; i < 4; ++i)
			{
				if (msg->jointgaittablestatus(i))
				{
					this->jointAngleShouldBe[i] = msg->jointgaittable(i);
				}
			}
			cout<<"Model: "<<model->GetName()<<":joint0:"<<jointAngleShouldBe[0]<<endl;
			cout<<"Model: "<<model->GetName()<<":joint1:"<<jointAngleShouldBe[1]<<endl;
			cout<<"Model: "<<model->GetName()<<":joint2:"<<jointAngleShouldBe[2]<<endl;
			cout<<"Model: "<<model->GetName()<<":joint3:"<<jointAngleShouldBe[3]<<endl;
			commandPub->Publish(feed_back_message);
			break;
		}
		case 4:
		{
			this->executionSate = 3;
			for (int i = 0; i < 4; ++i)
			{
				if (i==0 || i==3)
				{
					if (msg->jointgaittablestatus(i))
					{
						this->jointAngleShouldBe[i] = msg->jointgaittable(i);
					}
				}else{
					if (msg->jointgaittablestatus(i) && i == 1)
					{
						lftWheelSpeed = msg->jointgaittable(i);
					}
					if (msg->jointgaittablestatus(i) && i == 2)
					{
						rgtWheelSpeed = msg->jointgaittable(i);
					}
				}
			}
			commandPub->Publish(feed_back_message);
			break;
		}
		// case 5:
		// {
		// 	jointWF->SetAngle(0,math::Angle(msg->jointgaittable(0)));
		// 	jointWL->SetAngle(0,math::Angle(msg->jointgaittable(1)));
		// 	jointWR->SetAngle(0,math::Angle(msg->jointgaittable(2)));
		// 	jointCB->SetAngle(0,math::Angle(msg->jointgaittable(3)));
		// 	cout<<"Model: Initial Joint Angle Set "<<endl;
		// 	break;
		// }
	}
	if (commandType == 0 || commandType == 5)
	{
		startExecution = false;
	}else{
		startExecution = true;
		commandPriority = msg->priority();
	}
}

//------------- Low level model control functions --------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to set the torque of a specified joint
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::SetJointAngleForce(physics::JointPtr CurrentJoint, int RotAxis, math::Angle AngleDesired)
{
	CurrentJoint->SetAngle(RotAxis, AngleDesired);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to control the joint angle by using a PID controller
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::JointPIDController(JointPlus &CurrentJoint, double AngleDesiredRad, double DesireSpeed) 	// Desired Speed is a percentage of the maximum speed
{
	double AngleError, AngleDiffError;
	double SwingSpeed;
	int RotAxis = 0;
	math::Angle AngleDesired(AngleDesiredRad);
	AngleError = (AngleDesired - CurrentJoint.JointAngleNow).Radian();

	AngleDiffError = AngleError - CurrentJoint.JointErrorHis;
	CurrentJoint.JointErrorAccu += AngleError;
	SwingSpeed = JointAngleKPID.x*AngleError + JointAngleKPID.y*CurrentJoint.JointErrorAccu + JointAngleKPID.z*AngleDiffError;
	// cout<<"SwingSpeed:"<<SwingSpeed<<endl;
	if (abs(SwingSpeed)> MaxiRotationRate*DesireSpeed)
	{
		SwingSpeed = SwingSpeed>0?MaxiRotationRate*DesireSpeed:(-MaxiRotationRate*DesireSpeed);
	}
	SetJointSpeed(CurrentJoint.JointX,RotAxis,SwingSpeed);

	CurrentJoint.JointErrorHis = AngleError;
}

void ModelController::JointAngleUpdateInJointPlus(void)
{
	jointWRP.JointAngleNow = GetJointAngle(jointWR,0);
	jointWLP.JointAngleNow = GetJointAngle(jointWL,0);
	jointWFP.JointAngleNow = GetJointAngle(jointWF,0);
	jointCBP.JointAngleNow = GetJointAngle(jointCB,0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to get a specified joint angle
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
math::Angle ModelController::GetJointAngle(physics::JointPtr CurrentJoint, int RotAxis)
{
	math::Angle CurrentJointAngle;
	CurrentJointAngle = CurrentJoint->GetAngle(RotAxis);
	return CurrentJointAngle;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to control the joint speed
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will set the rotation rate of the joint
// The unit of this speed is rad/s
void ModelController::SetJointSpeed(physics::JointPtr CurrentJoint, int RotAxis, double SpeedDesired)
{
	CurrentJoint->SetVelocity(RotAxis,SpeedDesired);
}

//------------- Low level model control functions END ----------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// These functions will be used to check whether the module has finished execute a command
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::JointAngleTracking(void)
{
	bool execution_finished_flag = true;
	if (abs(GetJointAngle(jointWF,0).Radian()-jointAngleShouldBe[0])>EXECUTIONERROR)
	{
		execution_finished_flag = false;
		// cout<<"Model: "<<model->GetName() <<" joint 0 difference "<<abs(GetJointAngle(jointWF,0).Radian()-jointAngleShouldBe[0])<<endl;
	}
	if (abs(GetJointAngle(jointWL,0).Radian()-jointAngleShouldBe[1])>EXECUTIONERROR)
	{
		execution_finished_flag = false;
		// cout<<"Model: "<<model->GetName() <<" joint 1 difference "<<abs(GetJointAngle(jointWL,0).Radian()-jointAngleShouldBe[1])<<endl;
	}
	if (abs(GetJointAngle(jointWR,0).Radian()-jointAngleShouldBe[2])>EXECUTIONERROR)
	{
		execution_finished_flag = false;
		// cout<<"Model: "<<model->GetName() <<" joint 2 difference "<<abs(GetJointAngle(jointWR,0).Radian()-jointAngleShouldBe[2])<<endl;
	}
	if (abs(GetJointAngle(jointCB,0).Radian()-jointAngleShouldBe[3])>EXECUTIONERROR)
	{
		execution_finished_flag = false;
		// cout<<"Model: "<<model->GetName() <<" joint 3 difference "<<abs(GetJointAngle(jointCB,0).Radian()-jointAngleShouldBe[3])<<endl;
	}
	if (execution_finished_flag)
	{
		command_message::msgs::CommandMessage feed_back_message;
		feed_back_message.set_messagetype(0);
		feed_back_message.set_priority(commandPriority);
		feed_back_message.set_stringmessage(model->GetName()+":finished");
		commandPub->Publish(feed_back_message);
	}
}

void ModelController::PositionTracking(void)
{
	bool execution_finished_flag = true;
	math::Vector2d module_pos(GetModelCentralCoor().pos.x,GetModelCentralCoor().pos.y);
	math::Vector2d desired_pos(targetPosition.pos.x,targetPosition.pos.y);
	if (module_pos.Distance(desired_pos)>2*EXECUTIONERROR)
	{
		execution_finished_flag = false;
		cout<<"Model: distance is : "<<module_pos.Distance(desired_pos)<<endl;
	}
	if (abs(targetPosition.pos.z-GetModelCentralCoor().rot.GetYaw())>(EXECUTIONERROR+0.002))
	{
		execution_finished_flag = false;
		cout<<"Model: angle difference is : "<<abs(targetPosition.pos.z-GetModelCentralCoor().rot.GetYaw())<<endl;
	}
	if (execution_finished_flag)
	{
		command_message::msgs::CommandMessage feed_back_message;
		feed_back_message.set_messagetype(0);
		feed_back_message.set_priority(commandPriority);
		feed_back_message.set_stringmessage(model->GetName()+":finished");
		commandPub->Publish(feed_back_message);
	}
}

//----------------------- Utility functions --------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to calculate the angular velocity of a revolute joint
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double ModelController::RevolutionSpeedCal(physics::JointPtr JointNTBC, const int AxisIndex)
{
	double ResSpeedRef;

	// In the API instruction, fucntion GetVelocity() returns the "rotation rate"
	// The unit of this rotation rate is "rad/s"
	ResSpeedRef = JointNTBC->GetVelocity(AxisIndex);
	//cout<<"ResSpeedRef = "<<ResSpeedRef<<" rad/s"<<endl;

	return ResSpeedRef;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to get the coordinates and direction of the current model
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
math::Pose ModelController::GetModelCentralCoor(void)
{
	math::Pose ModelPosition;
	physics::ModelState CurrentModelState(model);
	ModelPosition = CurrentModelState.GetPose();
	return ModelPosition;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to get the coordinates and direction of the current model
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
math::Angle ModelController::AngleCalculation2Points(math::Vector2d StartPoint, math::Vector2d EndPoint)
{
	double displacementX, displacementY, angleC;
	displacementX = EndPoint.x - StartPoint.x;
	displacementY = EndPoint.y - StartPoint.y;
	// cout<<"direction vector is [ " << displacementX << "," <<displacementY<<"]"<<endl;
	angleC = atan2(displacementY,displacementX);
	math::Angle ReturnAngle(angleC+PI/2);
	return ReturnAngle;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to apply a complementary filter
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double ModelController::ComplementaryFilter(double FilteringValue)
{
	static double ValueRecorder = 0;
	double ComplementFilterPar = 0.9;
	double FiltedValue;
	FiltedValue = (1 - ComplementFilterPar)*FilteringValue + ComplementFilterPar*ValueRecorder;

	ValueRecorder = FiltedValue;

	return FiltedValue;
}

double ModelController::ComplementaryFilter(double FilteringValue, 
		double ComplementFilterPar)
{
	static double ValueRecorder = 0;
	double FiltedValue;
	FiltedValue = (1 - ComplementFilterPar)*FilteringValue + ComplementFilterPar*ValueRecorder;

	ValueRecorder = FiltedValue;

	return FiltedValue;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to return asked JointPlus
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
JointPlus & ModelController::GetJointPlus(int node_ID)
{
	if (node_ID == 1)
	{
		return jointWLP;
	}
	if (node_ID == 2)
	{
		return jointWRP;
	}
	if (node_ID == 3)
	{
		return jointCBP;
	}
	return jointWFP;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to return axis of a joint
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int ModelController::GetJointAxis(int node_ID)
{
	int axis_idx;
	switch(node_ID)
	{
		case 0:axis_idx = 1;break;
		case 1:axis_idx = 0;break;
		case 2:axis_idx = 0;break;
		case 3:axis_idx = 0;break;
	}
	return axis_idx;
}

//----------------------- Utility functions END ----------------------

//----------- Planar Motion Control functions ------------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to control the planar motion direction of SMORES on the ground
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::AnglePIDController(math::Angle DesiredAngle, math::Angle CurrentAngle, math::Vector2d CurrentSpeed)
{
	double AngleError, AngleErrorDiff, DiffSpeedControl;
	static double AngleErrorHis = 0;
	static double AngleErrorAcu = 0;
	double LeftWheelSpeed, RightWheelSpeed;

	while (abs(DesiredAngle.Degree()) > 180)
	{
		double DegreeAngle;
		DegreeAngle = DesiredAngle.Degree()>0?DesiredAngle.Degree()-360:DesiredAngle.Degree()+360;
		DesiredAngle.SetFromDegree(DegreeAngle);
	}
	// cout<<"Desired Angle is "<< DesiredAngle.Degree()<<endl;

	if (abs((DesiredAngle-CurrentAngle).Degree())>180)
	{
		AngleError = (DesiredAngle-CurrentAngle).Degree()>0?(DesiredAngle-CurrentAngle).Radian()-2*PI:(DesiredAngle-CurrentAngle).Radian()+2*PI;
	}else{
		AngleError = (DesiredAngle-CurrentAngle).Radian();
	}
	AngleErrorAcu += AngleError;
	AngleErrorDiff = AngleError - AngleErrorHis;
	DiffSpeedControl = ModelAngleKPID.x*AngleError + ModelAngleKPID.y*AngleErrorAcu + ModelAngleKPID.z*AngleErrorDiff;
	LeftWheelSpeed = CurrentSpeed.x - DiffSpeedControl;
	RightWheelSpeed = CurrentSpeed.y + DiffSpeedControl;

	// Maximium speed checking
	// if (abs(RightWheelSpeed)>jointWRP.MaximiumRotRate)
	// {
	// 	RightWheelSpeed = RightWheelSpeed>0?jointWRP.MaximiumRotRate:-jointWRP.MaximiumRotRate;
	// 	LeftWheelSpeed = RightWheelSpeed - 2*DiffSpeedControl;
	// 	if (abs(LeftWheelSpeed)>jointWLP.MaximiumRotRate)
	// 	{
	// 		LeftWheelSpeed = LeftWheelSpeed>0?jointWLP.MaximiumRotRate:-jointWLP.MaximiumRotRate;
	// 	}
	// }
	// if (abs(LeftWheelSpeed)>jointWLP.MaximiumRotRate)
	// {
	// 	LeftWheelSpeed = LeftWheelSpeed>0?jointWLP.MaximiumRotRate:-jointWLP.MaximiumRotRate;
	// 	RightWheelSpeed = LeftWheelSpeed + 2*DiffSpeedControl;
	// 	if (abs(RightWheelSpeed)>jointWRP.MaximiumRotRate)
	// 	{
	// 		RightWheelSpeed = RightWheelSpeed>0?jointWRP.MaximiumRotRate:-jointWRP.MaximiumRotRate;
	// 	}
	// }

	SetJointSpeed(jointWR, 0, RightWheelSpeed);
	SetJointSpeed(jointWL, 0, LeftWheelSpeed);
	AngleErrorHis = AngleError;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to control the model drive to a specofc point on the ground plane
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::Move2Point(math::Vector2d DesiredPoint, math::Angle DesiredOrientation)
{
	math::Pose CurrentPosition;
	math::Vector2d startPoint;
	math::Vector2d CurrentSpeed;
	CurrentPosition = GetModelCentralCoor();
	startPoint.x = CurrentPosition.pos.x;
	startPoint.y = CurrentPosition.pos.y;
	math::Angle desireAngle = AngleCalculation2Points(startPoint, DesiredPoint);
	double CurrentDistance = startPoint.Distance(DesiredPoint);
	if (CurrentDistance > PlanarMotionStopThreshold)
	{
		CurrentSpeed.x = AccelerationRate*CurrentDistance;
		if (CurrentSpeed.x > MaxiRotationRate)
		{
			CurrentSpeed.x = MaxiRotationRate;
		}
		CurrentSpeed.y = CurrentSpeed.x;
		AnglePIDController(desireAngle, CurrentPosition.rot.GetYaw(), CurrentSpeed);
	}else{
		CurrentSpeed.x = 0;
		CurrentSpeed.y = 0;
		AnglePIDController(DesiredOrientation, CurrentPosition.rot.GetYaw(), CurrentSpeed);
	}
}

//----------- Planar Motion Control functions END --------------------
GZ_REGISTER_MODEL_PLUGIN(ModelController)
} // namespace gazebo