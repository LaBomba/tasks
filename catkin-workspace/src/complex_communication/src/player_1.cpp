#include <cstdlib>
#include <algorithm>
#include <vector>
#include <boost/array.hpp>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "complex_communication/Turn.h"
#include "complex_communication/Table.h"

#define BOARD_SIZE 9

class Player
{
  public:
    Player(std::string name, int player_id, ros::Publisher pub);
    ~Player() {};

    bool isGameOver() const { return game_over_; };
    int makeMove(boost::array<int, BOARD_SIZE> board);
    void makeMoveCallback(const complex_communication::TableConstPtr& message);
    std::string getName() const { return name_; };
    int getID() const { return player_id_; };
    void setGameStatus(bool status) { game_over_ = status; };
    void closeCallback(const std_msgs::Int32 message);
  private:
    std::string name_;
    int player_id_;
    bool game_over_;
    ros::Publisher publisher_;
};

Player::Player(std::string name, int player_id, ros::Publisher pub)
{
  name_ = name;
  player_id_ = player_id;
  game_over_ = false;
  publisher_ = pub;
}

int Player::makeMove(boost::array<int, BOARD_SIZE>  board)
{
  // Check for free spots
  std::vector<int> temp;
  for (int i = 0; i < BOARD_SIZE; i++)
  {
    if (board[i] == 0)
    {
      temp.push_back(i);
    }
  }

  if (temp.size() == 0)
  {
    return 0;
  }

  std::random_shuffle(temp.begin(), temp.end());

  return temp.at(0);

}

void Player::makeMoveCallback(const complex_communication::TableConstPtr& message)
{
  if (message->play == Player::getID())
  {
    ROS_INFO("Now it's my turn.");

    int move = Player::makeMove(message->table);

    ROS_INFO("I play %d", move);

    // Sending move to server
    complex_communication::Turn turn;
    turn.spot = move;
    turn.id = Player::getID();

    publisher_.publish(turn);

  }

}

void Player::closeCallback(const std_msgs::Int32 message)
{
  if (message.data == 1)
  {
    Player::setGameStatus(true);

  }
}

int main(int argc, char** argv)
{
  const char* player_name = "Player1";
  const char* table_topic = "/task2/table";
  const char* play_topic = "/task2/play";
  const char* close_topic = "/task2/close";
  uint32_t queue_size = 200;

  ros::init(argc, argv, player_name);

  ros::NodeHandle node_handler;

  ros::Publisher publisher = \
                 node_handler.advertise<complex_communication::Turn>(play_topic, queue_size);

  Player player = Player(player_name, 1, publisher);

  ros::Subscriber subscriber = node_handler.subscribe(table_topic, queue_size,
                                               &Player::makeMoveCallback, &player);
  ros::Subscriber close_sub = node_handler.subscribe(close_topic, queue_size,
                                              &Player::closeCallback, &player);

  ros::Rate loop_rate(1);
  while (ros::ok() && !player.isGameOver())
  {

    ros::spinOnce();

    loop_rate.sleep();
  }

  ROS_INFO("Shutting down...", player.getID());

  return 0;
}
