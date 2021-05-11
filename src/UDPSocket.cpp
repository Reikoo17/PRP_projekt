//
// Created by ricky on 19.04.21.
//

#include "../include/UDPSocket.h"

std::string UDPSocket::string_to_nmea_message(const std::string& message){

    std::stringstream stream;

    uint16_t sum = get_message_checksum(message);

    stream << "$" << message << "*" << std::hex << sum;

    return stream.str();
}

uint16_t UDPSocket::get_message_checksum(const std::string& message){
    uint16_t sum = 0;
    for(char i : message){
        sum = sum ^ i;
    }
    return sum;
}

std::string UDPSocket::extract_nmea_message_content(const std::string& nmea_message){

    std::string edit_message = nmea_message.substr(1,nmea_message.find('*')-1);

    return edit_message;
}

bool UDPSocket::is_nmea_message_valid(const std::string& nmea_message){

    if (nmea_message.front() != '$') return false;

    std::string edit_message = extract_nmea_message_content(nmea_message);

    std::string s_number = nmea_message.substr(nmea_message.find('*')+1,2);
    uint16_t nmea_number = std::stoi(s_number,nullptr,16);

    if (get_message_checksum(edit_message) != nmea_number) return false;

    return true;
}

size_t UDPSocket::send_message(const std::string &aMessage) const{
    std::string edit_message = string_to_nmea_message(aMessage);
    size_t no_of_sent_bytes = send(c_socket, edit_message.c_str(), BUFFER, 0);

#ifdef DEBUG
    std::cout << "\nTvar zpravy: \n" << edit_message <<"\nPoslano bytu: "  << no_of_sent_bytes << std::endl;
#endif
    return no_of_sent_bytes;
}

std::string UDPSocket::receive_message() const{
    char buffer[BUFFER];
    std::string ret_string;

    int no_byte = recv(s_socket, buffer, sizeof(buffer), 0);

    std::string str_buffer(buffer);
    //std::cout << str_buffer << std::endl;
    if (!is_nmea_message_valid(str_buffer)) return "";

    ret_string = extract_nmea_message_content(str_buffer);

#ifdef DEBUG
    std::cout << "\nNeupravena verze: \n" << str_buffer << "\nUpravena verze: \n" << ret_string;
#endif
    return ret_string;
}

void UDPSocket::sender() {

    while(thread_sender) {

        if (!queue_send.empty()) {
            queue_send_lock.lock();
            std::string message = queue_send.front();
            queue_send.pop();
            queue_send_lock.unlock();
            //std::cout << "poslano: " << message << std::endl;
            if(send_message(message)==0) std::cout << "Zpráva nebyla poslána" << std::endl;
        }
       sleep_for(10us);
    }
}

void UDPSocket::receiver() {
    while(thread_receiver) {
        std::string message = receive_message();
        if (!message.empty()){
            queue_recv_lock.lock();
            //std::cout << "přijato: " << message << std::endl;
            queue_recv.push(message);
            queue_recv_lock.unlock();
        }
       sleep_for(10us);
    }
}

void UDPSocket::push_message(const std::string &message) {
    queue_send_lock.lock();
    queue_send.push(message);
    queue_send_lock.unlock();
}

std::string UDPSocket::get_message() {
    queue_recv_lock.lock();
    std::string message = queue_recv.front();
    queue_recv.pop();
    queue_recv_lock.unlock();

    return message;
}
