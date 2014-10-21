#include "ros/ros.h"
#include <cstdlib>

using namespace std;

class TicTacToe
{
  public:
    TicTacToe(int size);
    ~TicTacToe() {};

    bool isFull() const;
    void announceBoard() const;
    bool applyTurn(unsigned spot, int current_player);
    bool hasWinner();
    int getWinner() const;
  private:
    int* board_;
    int symbols_[2];
    int round_;
    int winner_;
    int board_size_;
    int next_player_;
    unsigned free_spots_;

};

TicTacToe::TicTacToe(int size)
{
  board_size_ = size;
  board_ = new int [size];

  // Assign a symbol for each player
  symbols_[0] = 1;
  symbols_[1] = 2;

  // Initialize game board
  for (int i = 0; i < board_size_; i++)
  {
    board_[i] = 0;
  }

  // Decide who plays first
  next_player_ = 1;

  // All the spots are free
  free_spots_ = board_size_;

  round_ = 0;
  winner_ = 0;
}

int TicTacToe::getWinner() const
{
  return winner_;
}

bool TicTacToe::hasWinner()
{
  // Check if we have a winner
  // Horizontal
  if (board_[0] == board_[1] &&  board_[1] == board_[2] && board_[0] != 0)
  {
    winner_ = board_[0];
    return true;
  }
  else if (board_[3] == board_[4] && board_[4] == board_[5] && board_[3] != 0)
  {
    winner_ = board_[3];
    return true;
  }
  else if (board_[6] == board_[7] && board_[7] == board_[8] && board_[6] != 0)
  {
    winner_ = board_[6];
    return true;
  }
  // Vertical
  else if (board_[0] == board_[3] && board_[3] == board_[6] && board_[0] != 0)
  {
    winner_ = board_[0];
    return true;
  }
  else if (board_[1] == board_[4] && board_[4] == board_[7] && board_[1] != 0)
  {
    winner_ = board_[1];
    return true;
  }
  else if (board_[2] == board_[5] && board_[5] == board_[8] && board_[2] != 0)
  {
    winner_ = board_[2];
    return true;
  }
  // Diagonal
  else if (board_[0] == board_[4] && board_[4] == board_[8] && board_[0] != 0)
  {
    winner_ = board_[0];
    return true;
  }
  else if (board_[2] == board_[4] && board_[4] == board_[6] && board_[2] != 0)
  {
    winner_ = board_[2];
    return true;
  }
  else
  {
    return false;
  }
}

bool TicTacToe::applyTurn(unsigned spot, int current_player)
{
  if (next_player_ == current_player)
  {
    // Check for validity
    if (board_[spot] == 0)
    {
      // The spot if free so add new move
      board_[spot] = current_player;

      // Change next player
      if (current_player == symbols_[0])
      {
        next_player_ = symbols_[1];
      }
      else if (current_player == symbols_[1])
      {
        next_player_ = symbols_[0];
      }

      round_++;
      free_spots_--;

      return true;
    }
    else
    {
      ROS_INFO("The move %d has already been played.", spot);
      return false;
    }
  }
  else
  {
    ROS_INFO("Wait for your turn, %d", current_player);
    return false;
  }
}

bool TicTacToe::isFull() const
{
  return (free_spots_ == 0);
}

void TicTacToe::announceBoard() const
{
  ROS_INFO("** Round: %d", round_);
  ROS_INFO("** Free spots: %d", free_spots_);
  for (int i = 0; i < board_size_; i++)
  {
    if ((i + 1) % 3 != 0)
    {

      ROS_INFO( " %d %d %d", board_[i], board_[i + 1], board_[i + 2]);
      i += 2;
    }
    else
    {
      ROS_INFO( "  %d\n",  board_[i]);
    }
  }
  ROS_INFO(" -----");
}


int main(int argc, char** argv)
{
  const char* server_name = "tictac";
  const char* table_topic = "/task2/table";
  const char* play_topic = "/task2/play";
  uint32_t queue_size = 200;
  int publish_rate = 1;

  ros::init(argc, argv, "name");

  ros::NodeHandle node_handler;

  ros::Rate loop_rate(publish_rate);
  TicTacToe game = TicTacToe(9);
  while(ros::ok())
  {
    game.announceBoard();

    ros::spinOnce();

    loop_rate.sleep();
  }

}
