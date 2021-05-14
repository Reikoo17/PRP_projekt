//
// Created by ricky on 19.04.21.
//
/** \file Robot.h
 * \brief Knihovna Robot
 * Knihovna pro definici parametrů robota a jeho řízení
 * \author Roman Štrobl, Vojtěch Strašil
 * \bug No known bugs.
*/
#ifndef PRP_PROJEKT_ROBOT_H
#define PRP_PROJEKT_ROBOT_H

#include <chrono>
#include <thread>
#include <regex>
#include <iostream>
#include <ros/ros.h>
#include <UDPSocket.h>
#include <fstream>
#include <RvizCommunication.h>

/**\brief Struktura kola
 * \details Struktura, která v sobě ukládá parametry kola pro řízení.
 * \param command pro výběr kola, které bude řídit. Tvar "RSPEED" nebo "LSPEED".
 * \param float actual_speed, parametr jakou rychlostí jede dané kolo
 * \param float set_speed, parametr pro nastavení rychlosti kola.
 * \param acceleration, parametr akcelerace kola. Tato hodnota se nastavuje podle rychlosti generátoru ramp.
 */
struct TWheel{
    std::string command;
    float actual_speed;
    float set_speed;
    float acceleration;
};

using namespace std::this_thread;     // sleep_for, sleep_until
using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
using std::chrono::system_clock;

enum Side{Left,Right};

class Robot :UDPSocket {
private:
//TODO: zbytečné info, buď použít u ramp, nebo smazat
    float ideg_step;
    float in_step;
    float in_ustep;

    float sensor[6]{};
    int LODO = 0;
    int RODO = 0;

    bool thread_wheels = false;
    bool thread_information = false;
    bool thread_zpracovavac = false;
    bool thread_regulace = false;

    TWheel iWRight{};
    TWheel iWLeft{};

    Data sData{sensor, iWRight.actual_speed, iWLeft.actual_speed};

public:
    Robot(float adeg_step, float ainterpol, int argc, char *argv[]): UDPSocket(8081,8080) {
        ideg_step = adeg_step;
        in_step = 360 / ideg_step;
        in_ustep = in_step * ainterpol;
        iWRight = {"RSPEED", 0, 0, 40};
        iWLeft = {"LSPEED", 0, 0, 40};

        ros::init(argc, argv, "Auto_node");       // connects node with ros core
        auto node = ros::NodeHandle();                            // API for ros functionality
        auto example_class = RosCommunication(node,"tf" ,"text", 10,sData);



        push_message("RESET,1");

        thread_wheels = true;
        thread_information = true;
        thread_zpracovavac = true;
        thread_regulace = true;

        //std::thread left(&Robot::RampGenerator,this,std::ref(iWLeft));
        //std::thread right(&Robot::RampGenerator,this,std::ref(iWRight));
        std::thread kola(&Robot::RampGenerator,this);

        //std::thread menu(&Robot::Realtime_control,this); // odkomentova, když chceš psát do konzole [LSPEED,RSPEED]
        std::thread information(&Robot::Info,this);
        std::thread zpracovac(&Robot::Message_proccesing,this);
        std::thread regulace(&Robot::Regulor,this);

        join_thread();
        //left.join();
        //right.join();
        kola.join();
        information.join();
        zpracovac.join();
        regulace.join();

    };

    virtual ~Robot() {
        ideg_step   = 0;
        in_step     = 0;
        in_ustep    = 0;

        thread_wheels = false;
        thread_information = false;
        thread_zpracovavac = false;
        thread_regulace = false;
    };

private:
    /**\brief  RampGenerator
      * \details Metoda která řídí jednotlivá kola robota pomocí generátoru rampy. Tato metoda funguje v samostatném vláknu.
      * \param[in] aWheel
   */
    //void RampGenerator(TWheel &aWheel);
    void RampGenerator();
    /**\brief  Info
      * \details Metoda, která generuje za určitý čas příkazy pro získání hodnot senzorů z robota. Tato metoda funguje v samostatném vláknu.
   */
    void Info();

    /**\brief  Message_proccesing
      * \details Metoda, která přijímá zprávy a hned je zpracovává. Tato metoda funguje v samostatném vláknu.
   */
    void Message_proccesing();

    /**\brief  Regulor
      * \details Tato metoda pomocí regulátoru řídí pohyb robota. Tato metoda funguje v samostatném vláknu.
   */
    void Regulor();

    /**\brief  SetSpeed
      * \details Metoda pro nastavení rychlosti na jednotlivých kolech
      * \param[in] aSide
      * \param[in] aSpeed
   */
    void SetSpeed(Side aSide, float aSpeed);
};

#endif //PRP_PROJEKT_ROBOT_H
