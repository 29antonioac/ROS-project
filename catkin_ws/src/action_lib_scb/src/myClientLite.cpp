#include <unistd.h>
#include <ctime>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv) {

	ros::init(argc,argv, "client_node");

	//crear el "action client"
	// true hace que el cliente haga "spin" en su propia hebra
	//Ojo: "move_base" puede ser cualquier string, pero hay que pensarlo como un "published topic". Por tanto, es importante que
	// el action server con el que va a comunicar tenga el mismo topic. E.d., al llamar a "as(n,string,....)" en el lado del
	// action server hay que usar el mismo string que en el cliente.
	MoveBaseClient ac("mi_move_base", true);  //<- poner "mi_move_base" para hacer mi propio action server.

	//Esperar 60 sg. a que el action server esté levantado.
	ROS_INFO("Esperando a que el action server mi_move_base se levante");
	//si no se conecta, a los 60 sg. se mata el nodo.
	ac.waitForServer(ros::Duration(60));

	ROS_INFO("Conectado al servidor mi_move_base");

	//Enviar un objetivo a move_base

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = 	"map";
	goal.target_pose.header.stamp =	ros::Time::now();

	goal.target_pose.pose.position.x =	-18.174;
	goal.target_pose.pose.position.y =	25.876;
	goal.target_pose.pose.orientation.w =	1;

	ROS_INFO("Enviando el objetivo");
	ac.sendGoal(goal);


	//Esperar al retorno de la acción
	ROS_INFO("Esperando al resultado  de la acción");
	ac.waitForResult();


	ROS_INFO("Nuevo ENTRANDO EN EL WHIIIIIIIIIIIIIIIIIIIIIIIIILEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
	while(ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
		ROS_INFO("Nuevo DENTRO DEL WHIIIIIIIIIIIIIIIIIIIIIIIIILEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
		move_base_msgs::MoveBaseGoal randomGoal;

		randomGoal.target_pose.header.frame_id = 	"map";
		randomGoal.target_pose.header.stamp =	ros::Time::now();

		randomGoal.target_pose.pose.position.x =	rand() % 40 - 20;
		randomGoal.target_pose.pose.position.y =	rand() % 40 - 20;
		randomGoal.target_pose.pose.orientation.w =	1;

		ROS_INFO("Enviando nuevo goal.");

		ac.sendGoal(randomGoal);

		// // Estar 10 segundos con el nuevo objetivo aleatorio
		// ROS_INFO("Nuevo. Antes del sleep");
		// sleep(10);
		// ROS_INFO("Nuevo. Después del sleep");
		//
		// // Cancelar el objetivo aleatorio
		// ac.cancelGoal();
		bool finished = ac.waitForResult(ros::Duration(10));

		if(!finished){
			ac.cancelGoal();
		}

		// Reiniciar el objetivo inicial
		ac.sendGoal(goal);

		//Esperar al retorno de la acción
		ROS_INFO("Esperando al resultado  de la acción");
		ac.waitForResult();
	}

	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("¡¡ Objetivo alcanzado !!");
	else ROS_INFO("Se ha fallado por alguna razón.");

	return 0;

}
