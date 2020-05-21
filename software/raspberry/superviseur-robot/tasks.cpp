/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TMANAGECOMROBOT 23
#define PRIORITY_TSTARTROBOT 23
#define PRIORITY_TSTARTWDRELOADER 26
#define PRIORITY_TMOVE 20
#define PRIORITY_TBATTERY 19
#define PRIORITY_TRESETSUPERVISOR 29
#define PRIORITY_TRESETROBOT 28
/*Fonctionnalités 14 à 19
#define PRIORITY_TCAMERA 21
*/

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_compteurEchec, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_withWD, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    /*Fonctionnalités 14 à 19
    if (err = rt_mutex_create(&mutex_allowCapture, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_image, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_allowCapturePosition, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arena, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_arenaIsCorrect, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    */
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_comRobotClosed, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_resetSupervisor, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_resetRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startWDReloader, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_acceptNextMonitor, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    /*Fonctionnalités 14 à 19
    if (err = rt_sem_create(&sem_openCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_closeCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_findArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_confirmArena, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_findPosition, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_stopFindPosition, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    */
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_manageComRobot, "th_manageComRobot", 0, PRIORITY_TMANAGECOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startWDReloader, "th_startWDReloader", 0, PRIORITY_TSTARTWDRELOADER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_resetSupervisor, "th_resetSupervisor", 0, PRIORITY_TRESETSUPERVISOR, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_resetRobot, "th_resetRobot", 0, PRIORITY_TRESETROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_manageComRobot, (void(*)(void*)) & Tasks::ManageComRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startWDReloader, (void(*)(void*)) & Tasks::StartWDReloaderTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_resetSupervisor, (void(*)(void*)) & Tasks::ResetSupervisorTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_resetRobot, (void(*)(void*)) & Tasks::ResetRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
    
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    while(1)
    {
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        status = monitor.Open(SERVER_PORT);
        rt_mutex_release(&mutex_monitor);

        cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

        if (status < 0) throw std::runtime_error {
            "Unable to start server on port " + std::to_string(SERVER_PORT)
        };
        monitor.AcceptClient(); // Wait the monitor client
        cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
        rt_sem_broadcast(&sem_serverOk);
        
        rt_sem_p(&sem_acceptNextMonitor, TM_INFINITE);
    }
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    Message *reponse;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    while(1){
        rt_sem_p(&sem_serverOk, TM_INFINITE);
        cout << "Received message from monitor activated" << endl << flush;

        while (!((msgRcv = monitor.Read())->CompareID(MESSAGE_MONITOR_LOST))){
            
            cout << "Rcv <= " << msgRcv->ToString() << endl << flush;
            if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
                rt_sem_v(&sem_openComRobot);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_CLOSE)) {
                rt_sem_v(&sem_closeComRobot);
                //if(rt_sem_p(&sem_comRobotClosed, )==ETIMEDOUT)


            } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
                rt_mutex_acquire(&mutex_withWD, TM_INFINITE);
                withWD = false;
                rt_mutex_release(&mutex_withWD);
                rt_sem_v(&sem_startRobot);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
                rt_mutex_acquire(&mutex_withWD, TM_INFINITE);
                withWD = true;
                rt_mutex_release(&mutex_withWD);
                rt_sem_v(&sem_startRobot);
            }  else if (msgRcv->CompareID(MESSAGE_ROBOT_RESET)) {
                rt_sem_v(&sem_resetRobot);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msgRcv->GetID();
                rt_mutex_release(&mutex_move);
            }
            delete(msgRcv); // mus be deleted manually, no consumer
        }
        cout << "Monitor is lost" << endl << flush;
        rt_sem_v(&sem_resetSupervisor);
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::ManageComRobotTask(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        //status = robot.Open("92.184.108.216",6699);
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
            WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
            /*ComRobot is opened, wait on the close of the commuictation*/
        
            rt_sem_p(&sem_closeComRobot, TM_INFINITE);
            rt_sem_v(&sem_resetRobot);
            cout << "Close serial com (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            status = robot.Close();
            rt_mutex_release(&mutex_robot);
            cout << status;
            cout << ")" << endl << flush;
             if (status < 0) {
                msgSend = new Message(MESSAGE_ANSWER_NACK);
            } else {
                msgSend = new Message(MESSAGE_ANSWER_ACK);
            }
            WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
        }        
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if(robotStarted == 0){
            rt_mutex_acquire(&mutex_withWD, TM_INFINITE);
            bool localWithWD= withWD;
            rt_mutex_release(&mutex_withWD);
            if(localWithWD)
            {
                cout << "Start robot with watchdog (";
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                msgSend = robot.Write(robot.StartWithWD());
                rt_mutex_release(&mutex_robot);
                rt_sem_v(&sem_startWDReloader);
            }
            else
            {
                cout << "Start robot without watchdog (";
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                msgSend = robot.Write(robot.StartWithoutWD());
                rt_mutex_release(&mutex_robot);
            }
            CheckRobotAnswers(msgSend);
            cout << msgSend->GetID();
            cout << ")" << endl;
            cout << "Movement answer: " << msgSend->ToString() << endl << flush;
            WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                if(localWithWD)
                    robotStarted = 2;
                else
                    robotStarted = 1;
            }
        }
        rt_mutex_release(&mutex_robotStarted);
        
    }
}

/**
 * @brief Thread in charge of reloading Robot's Watchdog. (periodicity is normally 1 sec (10^9))
 */
void Tasks::StartWDReloaderTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    /**************************************************************************************/
    /* The task startWDReloader starts here                                                    */
    /**************************************************************************************/
    while(1)
    {
        
        rt_sem_p(&sem_startWDReloader, TM_INFINITE);
        rt_task_set_periodic(NULL, TM_NOW, 1000000000);
        while (1) {
            rt_task_wait_period(NULL);
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            if(robotStarted!=2)
            {
                rt_mutex_release(&mutex_robotStarted);
                break;
            }
            rt_mutex_release(&mutex_robotStarted);
            cout << "Periodic WatchDog Reload update";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(robot.ReloadWD());
            rt_mutex_release(&mutex_robot);
            cout << endl << flush; 
            
        }
    }
    
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    Message *reponse;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs != 0) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            reponse = robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
            CheckRobotAnswers(reponse);
        }
        cout << endl << flush;
    }
}

/**
 * @brief Thread that refresh the current robot's battery level the robot.
 */
void Tasks::BatteryTask(void *arg) {
    int rs;
    Message* batteryLevel;
    MessageID batteryLevelID;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task battery starts here                                                    */
    /**************************************************************************************/
    
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    
    while(1){
        rt_task_wait_period(NULL);
        cout << "Periodic battery level check";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        // Check if the robot has started
        
        if (rs != 0){
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            batteryLevel = robot.Write(robot.GetBattery());
            rt_mutex_release(&mutex_robot);
            CheckRobotAnswers(batteryLevel);
            batteryLevelID = batteryLevel->GetID();
            // Write the battery level in the q_messageToMon queue
            if(batteryLevelID == MESSAGE_ROBOT_BATTERY_LEVEL){
                WriteInQueue(&q_messageToMon,batteryLevel);
            }   
        }
        cout << endl << flush;
    }
}

/**
 * @brief Thread that resets the supervisor (the supervisor get back to inital state)
 */
void Tasks::ResetSupervisorTask(void *arg) {
    int err;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task resetSupervisor starts here                                                    */
    /**************************************************************************************/
    while(1)
    {
        rt_sem_p(&sem_resetSupervisor, TM_INFINITE);
        cout << "Rebooting the server..."<< endl << flush;
        cout << "->Closing ComRobot..."<< endl << flush;
        rt_sem_v(&sem_closeComRobot);
        rt_mutex_acquire(&mutex_compteurEchec, TM_INFINITE);
        compteurEchec = 0;
        rt_mutex_release(&mutex_compteurEchec);
        cout << "->Closing comMonitor..."<< endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Close();
        rt_mutex_release(&mutex_monitor);
        cout << "->Allowing the supervisor to accept a new supervisor"<< endl << flush;
        rt_sem_v(&sem_acceptNextMonitor);
    }
}

/**
 * @brief Thread that stops and resets the robot(the supervisor get back to pre-start state)
 */
void Tasks::ResetRobotTask(void *arg) {
    Message *reponse;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task resetRobot starts here                                                    */
    /**************************************************************************************/
    while(1)
    {
        rt_sem_p(&sem_resetRobot, TM_INFINITE);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 0;
        rt_mutex_release(&mutex_robotStarted);
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        //reponse = robot.Write(robot.Stop());
        reponse = robot.Write(robot.Reset());
        rt_mutex_release(&mutex_robot);
        rt_mutex_acquire(&mutex_move, TM_INFINITE);
        move = MESSAGE_ROBOT_STOP;
        rt_mutex_release(&mutex_move);
        rt_mutex_acquire(&mutex_withWD, TM_INFINITE);
        withWD = false;
        rt_mutex_release(&mutex_withWD);
    }
}

/**
 * Checks the robots answer (after robot.write), changes the error counter
 * Verify that the comunication with the robot stay alive (less than 3 answer errors in a row)
 * @param reponse Answer Message to be checked
 */
void Tasks::CheckRobotAnswers(Message *reponse){
    if(reponse->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)||
                    reponse->CompareID(MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND)||
                    reponse->CompareID(MESSAGE_ANSWER_ROBOT_ERROR) ||
                    reponse->CompareID(MESSAGE_ANSWER_COM_ERROR)){
        if(compteurEchec<2)
            compteurEchec++;
        else
        {
            WriteInQueue(&q_messageToMon, new Message((MessageID)MESSAGE_ANSWER_COM_ERROR));
            rt_sem_v(&sem_closeComRobot);
            compteurEchec=0;
        }
    }
    else
    {
        compteurEchec=0;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

