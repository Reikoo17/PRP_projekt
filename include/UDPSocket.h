//
// Created by ricky on 19.04.21.
//
#ifndef PRP_PROJEKT_UDPSOCKET_H
#define PRP_PROJEKT_UDPSOCKET_H

/** \file communication.h
 * \brief Komunikační knihovna
 * Komunikační knihovna, která komunikuje se simulací robota.
 * \author Roman Štrobl
 * \bug No known bugs.
*/

#include <string>
#include <iostream>
#include <sstream>
#include <chrono>

#include <queue>
#include <thread>
#include <mutex>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define BUFFER 100
//#define DEBUG
using namespace std::this_thread;     // sleep_for, sleep_until
using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
using std::chrono::system_clock;

/**\brief Třída communicatio
 * \details Třída, pomocí které lze vytvořit socket pro komunikaci se simulací robota.
 */
class UDPSocket {

    int c_socket, s_socket;
    int c_res, s_res;

    struct  sockaddr_in c_adress{};
    struct  sockaddr_in s_adress{};

    std::queue<std::string> queue_send;
    std::queue<std::string> queue_recv;

    bool thread_sender = true;
    bool thread_receiver = true;

    std::thread sending_thread = std::thread(&UDPSocket::sender, this);
    std::thread receiving_thread = std::thread(&UDPSocket::receiver,this);

    std::mutex queue_send_lock;
    std::mutex queue_recv_lock;

public:

    UDPSocket(in_port_t as_port, in_port_t ac_port){
        s_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        c_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        s_adress.sin_family = AF_INET;
        s_adress.sin_addr.s_addr = htonl(INADDR_ANY);
        s_adress.sin_port = htons(as_port);

        c_adress.sin_family = AF_INET;
        c_adress.sin_addr.s_addr = inet_addr("127.0.0.1");
        c_adress.sin_port = htons(ac_port);

        s_res = bind(s_socket, (struct sockaddr *) &s_adress, sizeof(struct sockaddr_in));
        c_res = connect(c_socket, (struct sockaddr *) &c_adress, sizeof(struct sockaddr_in));

        //sending_thread.join();
        //receiving_thread.join();
    }

    ~UDPSocket(){
        
        thread_sender = false;
        thread_receiver = false;
    };

    void join_thread(){
        sending_thread.join();
        receiving_thread.join();
    }

    bool empty_box(){
        return queue_recv.empty();
    };

    void push_message(const std::string& message);

    std::string get_message();


private:
    /**\brief  string_to_nmea_message
       * \details Metoda pro převod příkazu na nmea zprávu.
       * \param[in] message
       * \return ret_string
    */
    static std::string string_to_nmea_message(const std::string& message);

    /**\brief  get_message_checksum
       * \details Kontrolní součet nmea zprávy.
       * \param[in] message
       * \param[in] sum
    */
    static uint16_t get_message_checksum(const std::string& message);

    /**\brief  extract_nmea_message_content
       * \details Metoda pro převod z nmea zprávy na string se kterým můžeme dále pracovat.
       * \param[in] message
       * \return edit_message
    */
    static std::string extract_nmea_message_content(const std::string& nmea_message);

    /**\brief  is_nmea_message_valid
       * \details Validace nmea zprávy.
       * \param[in] nmea_message
       * \return bool
    */
    static bool is_nmea_message_valid(const std::string& nmea_message);

    size_t send_message(const std::string &aMessage) const;

    std::string receive_message() const;

    void sender();

    void receiver();

};



#endif //PRP_PROJEKT_UDPSOCKET_H
