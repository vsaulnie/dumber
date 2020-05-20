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

#ifndef __TASKS_H__
#define __TASKS_H__

#include <unistd.h>
#include <iostream>

#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/queue.h>

#include "messages.h"
#include "commonitor.h"
#include "comrobot.h"
#include "camera.h"
#include "img.h"

using namespace std;

class Tasks {
public:
    /**
     * @brief Initializes main structures (semaphores, tasks, mutex, etc.)
     */
    void Init();

    /**
     * @brief Starts tasks
     */
    void Run();

    /**
     * @brief Stops tasks
     */
    void Stop();
    
    /**
     * @brief Suspends main thread
     */
    void Join();
    
private:
    /**********************************************************************/
    /* Shared data                                                        */
    /**********************************************************************/
    ComMonitor monitor;
    ComRobot robot;
    int robotStarted = 0;
    int move = MESSAGE_ROBOT_STOP;
    int compteurEchec=0;
    bool withWD = false;
    /*Fonctionnalités 14 à 19
    bool allowCapture = false;
    Image *image = null;
    bool allowCapturePosition = false;
    Arena *arena = null;
    bool arenaIsCorrect = flase; 
     */
    
    
    /**********************************************************************/
    /* Tasks                                                              */
    /**********************************************************************/
    RT_TASK th_server;
    RT_TASK th_sendToMon;
    RT_TASK th_receiveFromMon;
    RT_TASK th_manageComRobot;
    RT_TASK th_startRobot;
    RT_TASK th_startWDReloader;
    RT_TASK th_move;
    RT_TASK th_battery;
    RT_TASK th_resetSupervisor;
    RT_TASK th_resetRobot;
    
    /**********************************************************************/
    /* Mutex                                                              */
    /**********************************************************************/
    RT_MUTEX mutex_monitor;
    RT_MUTEX mutex_robot;
    RT_MUTEX mutex_robotStarted;
    RT_MUTEX mutex_move;
    RT_MUTEX mutex_compteurEchec;
    RT_MUTEX mutex_withWD;
    /* Fonctionnalités 14 à 19
    RT_MUTEX mutex_allowCapture;
    RT_MUTEX mutex_image;
    RT_MUTEX mutex_allowCapturePosition;
    RT_MUTEX mutex_arena;
    RT_MUTEX mutex_arenaIsCorrect;
    */

    /**********************************************************************/
    /* Semaphores                                                         */
    /**********************************************************************/
    RT_SEM sem_barrier;
    RT_SEM sem_serverOk;
    RT_SEM sem_openComRobot;
    RT_SEM sem_closeComRobot;
    RT_SEM sem_comRobotClosed;
    RT_SEM sem_startRobot;
    RT_SEM sem_resetSupervisor;
    RT_SEM sem_resetRobot;
    RT_SEM sem_startWDReloader;
    RT_SEM sem_acceptNextMonitor;
    /* Fonctionnalités 14 à 19
    RT_SEM sem_openCamera;
    RT_SEM sem_closeCamera;
    RT_SEM sem_findArena;
    RT_SEM sem_comfirmArena;
    RT_SEM sem_findPosition;
    RT_SEM sem_stopFindPosition;
    */
    
    /**********************************************************************/
    /* Message queues                                                     */
    /**********************************************************************/
    int MSG_QUEUE_SIZE;
    RT_QUEUE q_messageToMon;
    
    /**********************************************************************/
    /* Tasks' functions                                                   */
    /**********************************************************************/
    /**
     * @brief Thread handling server communication with the monitor.
     */
    void ServerTask(void *arg);
     
    /**
     * @brief Thread sending data to monitor.
     */
    void SendToMonTask(void *arg);
        
    /**
     * @brief Thread receiving data from monitor.
     */
    void ReceiveFromMonTask(void *arg);
    
    /**
     * @brief Thread opening communication with the robot.
     */
    void ManageComRobotTask(void *arg);

    /**
     * @brief Thread starting the communication with the robot.
     */
    void StartRobotTask(void *arg);
    
    /**
    * @brief Thread in charge of reloading Robot's Watchdog.
    */
    void StartWDReloaderTask(void *arg);
    
    /**
     * @brief Thread handling control of the robot.
     */
    void MoveTask(void *arg);
    
    /**
    * @brief Thread that refresh the current robot's battery level the robot.
    */
   void BatteryTask(void *arg);

   /**
    * @brief Thread that resets the supervisor (the supervisor get back to inital state)
    */
   void ResetSupervisorTask(void *arg);

   /**
    * @brief Thread that stops and resets the robot(the supervisor get back to pre-start state)
    */
   void ResetRobotTask(void *arg);

   /**
    * Checks the robots answer (after robot.write), changes the error counter
    * Verify that the comunication with the robot stay alive (less than 3 answer errors in a row)
    * @param reponse Answer Message to be checked
    */
   void CheckRobotAnswers(Message *reponse);
    
    /**********************************************************************/
    /* Queue services                                                     */
    /**********************************************************************/
    /**
     * Write a message in a given queue
     * @param queue Queue identifier
     * @param msg Message to be stored
     */
    void WriteInQueue(RT_QUEUE *queue, Message *msg);
    
    /**
     * Read a message from a given queue, block if empty
     * @param queue Queue identifier
     * @return Message read
     */
    Message *ReadInQueue(RT_QUEUE *queue);

};

#endif // __TASKS_H__ 

