//
// Created by ricky on 19.04.21.
//

#include "Robot.h"
#include <fstream>
using std::cout;


void Robot::RampGenerator(TWheel &aWheel) {

    while (thread_wheels) {
        std::stringstream stream;
            if ((aWheel.set_speed - aWheel.actual_speed) > 0) {
                aWheel.actual_speed += aWheel.acceleration;
            } else if ((aWheel.set_speed - aWheel.actual_speed) < 0){
                aWheel.actual_speed -= aWheel.acceleration;
            }
#ifdef DEBUG
            std::cout << aWheel.actual_speed << std::endl;
#endif
            stream << aWheel.command << "," << aWheel.actual_speed << std::flush;
            push_message(stream.str());
            stream.str("");
            sleep_for(1ms);
    }
}
void Robot::SetSpeed(Side aSide, float aSpeed) {
    if (aSide == 0){
        iWLeft.set_speed = aSpeed;
    }
    else {
        iWRight.set_speed = aSpeed;
    }
}

void Robot::Info(){
    std::stringstream stream;
    while(thread_information){

        stream << "LODO" << ","<< 1 << std::flush;
        push_message(stream.str());
        stream.str("");

        stream << "RODO" << ","<< 1 << std::flush;
        push_message(stream.str());
        stream.str("");

        for(size_t i = 0; i<3;++i) {
            stream << "SENSOR" << "," << i << std::flush;
            push_message(stream.str());
            stream.str("");
        }
        ros::spinOnce();
        sleep_for(800us);
       // ros::spinOnce();
    }


}

void Robot::Message_proccesing() {
    std::regex rgx("(\\w*),(\\d+),?(\\d*.\\d*)?");
    std::smatch matches;
    while (thread_zpracovavac) {
        if (!empty_box()) {
            std::string message = get_message();
            if (std::regex_search(message, matches, rgx)) {
                if (matches[1].str() == "LODO") {
                    LODO += stoi(matches[2]);
                } else if (matches[1].str() == "RODO") {
                    RODO += stoi(matches[2]);
                } else if (matches[1].str() == "SENSOR") {
                    sensor[stoi(matches[2])] = std::stof(matches[3]);
                }
            }
        }
        sleep_for(10us);
    }
}

void Robot::Regulor() {

    /*std::fstream file;
    std::fstream file1;
    std::fstream file2;
    file.open("odchylka_rovne.txt",std::ios::out);
    file1.open("pravy_motor_rovne.txt",std::ios::out);
    file2.open("levy_motor_rovne.txt",std::ios::out);*/

    //regulator
const int Kp = 220 ;//170
const int Ki = 1.2 ;//0.01
const int Kd = 2300 ;//100

    int error = 0;
    int lasterror = 0;
    int integral = 0;
    int derivate = 0;
    int offsetS = 50; //odchylka senzoru

    //ostatni promenne
    int stav = 0;
    int MIN0=100;
    int MAX0=0;
    int MIN1=100;
    int MAX1=0;
    int MIN2=100;
    int MAX2=0;
    int s1_hodnota = 0;
    int s0_hodnota = 0;
    int s2_hodnota = 0;

    int turn = 50; //otočka
    int offset = 6400; //výchozí rychlost motoru
    int PowerR = 0;
    int PowerL = 0;
    int PowerR_c = 0;
    int PowerL_c = 0;

    bool flag_calib = 1;
    int calib_pocet = 0;



    while(thread_regulace) {
        //cout << "Stav: "<< stav << std::endl
        //        << "senzor 1: "<< sensor[0] << std::endl
        //        << "senzor 2: "<< sensor[1] << std::endl
        //        << "senzor 3: "<< sensor[2] << std::endl;
        //sensory
        if(flag_calib){
            if(MAX1<=sensor[1])
                MAX1=sensor[1];

            if(MIN1>=sensor[1])
                MIN1=sensor[1];

            if(MAX0<=sensor[0])
                MAX0=sensor[0];

            if(MIN0>=sensor[0])
                MIN0=sensor[0];

            if(MAX2<=sensor[2])
                MAX2=sensor[2];

            if(MIN2>=sensor[2])
                MIN2=sensor[2];
        }

        s0_hodnota=(100-0)*((sensor[0]-MIN0)/(MAX0-MIN0));
        s1_hodnota=(100-0)*((sensor[1]-MIN1)/(MAX1-MIN1));
        s2_hodnota=(100-0)*((sensor[2]-MIN2)/(MAX2-MIN2));

        if (s0_hodnota < 4){
            s0_hodnota = 0;
        }
        if (s1_hodnota < 4){
            s1_hodnota = 0;
        }
        if (s2_hodnota < 4){
            s2_hodnota = 0;
        }


        //regulace
        error = offsetS - s1_hodnota;
        integral = integral + error;
        derivate = error - s1_hodnota;
        turn = (Kp*error) + (Ki*integral) + (Kd*derivate);
        turn = turn/130; //rychlost 400 dělono 1000 //rychlost 800 děleno 500 //1600 250 //3200 125
        //cout << "SENSOR: " << s1_hodnota << "\n";
        //cout << "TURN: " << turn << "\n";



        switch (stav){

            case 0: //kalibrace senzorů
                SetSpeed(Right, 2000);
                SetSpeed(Left, -2000);

                if (sensor[0]>3500){
                    stav = 1;
                }
                break;
            case 1:
                //SetSpeed(Right, 1000);
                //SetSpeed(Left, -1000);

                if (calib_pocet > 2){ //180 1 270 2
                    stav = 2;
                }
                else {
                    if (sensor[0]<1000){
                        calib_pocet = calib_pocet + 1;
                        stav = 0;
                    }
                }
                break;

            case 2:
                flag_calib = 0;
                SetSpeed(Right, -200);
                SetSpeed(Left, 200);

                if ((s1_hodnota<(offsetS+3))&&(s1_hodnota>(offsetS-3))){
                    cout << "START" << "\n";
                    error = 0;
                    lasterror = 0;
                    integral = 0;
                    derivate = 0;
                    stav = 10;
                }

                break;

//regulace
            case 10:

                PowerR=offset-turn;
                PowerL=offset+turn;


                if ((s0_hodnota>80)&&(s2_hodnota>80)&&(error < 10)){
                    cout << "CROSS" << "\n";
                    stav=20;
                }

                if ((s0_hodnota>80)&&(error > 30)){
                    cout << "MISSS0" << "\n";
                    stav=30;
                }

                if ((s2_hodnota>80)&&(error > 30)){
                    cout << "MISSS2" << "\n";
                    stav=40;
                }

                /*file << turn << "\n";
                file1 << PowerR<< "\n";
                file2 << PowerL << "\n";


                 if (RODO>45000){
                     SetSpeed(Right, 0);
                     SetSpeed(Left, 0);
                     file.close();
                     file1.close();
                     file2.close();
                     */

                SetSpeed(Right, PowerR);
                SetSpeed(Left, PowerL);

                break;

                //CROSS
            case 20:

                SetSpeed(Right, offset);
                SetSpeed(Left, offset);

                if ((s0_hodnota<20)&&(s2_hodnota<20)){
                    cout << "DONE" << "\n";
                    lasterror=0;
                    error=0;
                    stav=10;
                }

                break;

                //MISSS0
            case 30:


                SetSpeed(Right, (PowerR-(offset/3))/3);
                SetSpeed(Left, (PowerL+(offset/4))/3);

                if ((s1_hodnota<(offsetS+3))&&(s1_hodnota>(offsetS-3))){
                    cout << "DONE" << "\n";
                    lasterror=0;
                    error=0;
                    stav = 10;
                }

                break;

                //MISSS0
            case 40:


                SetSpeed(Right, (PowerR+(offset/4))/3);
                SetSpeed(Left, (PowerL-(offset/3))/3);

                if ((s1_hodnota<(offsetS+3))&&(s1_hodnota>(offsetS-3))){
                    cout << "DONE" << "\n";
                    lasterror=0;
                    error=0;
                    stav = 10;
                }

                break;




        }

        lasterror = error;


        sleep_for(125us);

    }}