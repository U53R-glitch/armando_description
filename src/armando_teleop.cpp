// armando_teleop: telecomando da tastiera per il braccio Armando.
// Legge i tasti, aggiorna la posizione desiderata dei giunti e la invia
// al joint_trajectory_controller come JointTrajectory con un solo punto.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// ---------------------------------------------------------------------------
// Terminale in modalita' "raw": i tasti arrivano subito, senza premere Invio
// e senza essere stampati a schermo. Il distruttore ripristina il terminale.
// ---------------------------------------------------------------------------
class RawTerminal
{
public:
  RawTerminal()
  {
    tcgetattr(STDIN_FILENO, &old_);          // salva le impostazioni originali
    termios raw = old_;
    raw.c_lflag &= ~(ICANON | ECHO);         // niente attesa di Invio, niente eco
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  }

  ~RawTerminal()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_); // rimette il terminale come prima
  }

  // Aspetta un tasto per al massimo timeout_ms millisecondi.
  // Restituisce il carattere premuto, oppure -1 se non e' arrivato nulla.
  int readKey(int timeout_ms)
  {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout_ms * 1000;
    if (select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv) > 0) {
      char c;
      if (read(STDIN_FILENO, &c, 1) == 1) {
        return c;
      }
    }
    return -1;
  }

private:
  termios old_;
};

// ---------------------------------------------------------------------------
// Il nodo ROS 2
// ---------------------------------------------------------------------------
class ArmandoTeleop : public rclcpp::Node
{
public:
  ArmandoTeleop()
  : Node("armando_teleop")
  {
    // Publisher: la "cassetta postale" del joint_trajectory_controller
    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory_controller/joint_trajectory", 10);

    // Subscriber: legge /joint_states per sapere da dove parte il braccio
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ArmandoTeleop::jointStateCallback, this, std::placeholders::_1));
  }

  bool ready() const {return initialized_;}

  // Traduce un tasto in un movimento e pubblica il nuovo obiettivo
  void handleKey(char key)
  {
    const std::string plus = "qwer";   // q,w,e,r -> j0..j3 positivo
    const std::string minus = "asdf";  // a,s,d,f -> j0..j3 negativo
    const std::vector<double> q_prev = q_;   // obiettivo prima del tasto

    if (key == 'h') {                  // h -> posa "home": tutti i giunti a zero
      std::fill(q_.begin(), q_.end(), 0.0);
    } else if (plus.find(key) != std::string::npos) {
      q_[plus.find(key)] += step_;
    } else if (minus.find(key) != std::string::npos) {
      q_[minus.find(key)] -= step_;
    } else {
      return;                          // tasto non previsto: non fare nulla
    }

    // Non chiedere mai posizioni oltre i limiti scelti
    for (size_t i = 0; i < q_.size(); ++i) {
      q_[i] = std::clamp(q_[i], lower_[i], upper_[i]);
    }

    // Durata del movimento: almeno move_time_, ma abbastanza lunga da non
    // superare la velocita' massima dei giunti (velocity="1.0" nell'URDF)
    double max_delta = 0.0;
    for (size_t i = 0; i < q_.size(); ++i) {
      max_delta = std::max(max_delta, std::abs(q_[i] - q_prev[i]));
    }
    const double duration = std::max(move_time_, max_delta / max_vel_);

    publishTarget(duration);
    std::printf("j0=%+.2f  j1=%+.2f  j2=%+.2f  j3=%+.2f   (t=%.2f s)\n",
      q_[0], q_[1], q_[2], q_[3], duration);
  }

private:
  // Chiamata a ogni messaggio su /joint_states; usa solo il primo.
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (initialized_) {
      return;
    }
    // In JointState l'ordine dei giunti non e' garantito: si cerca per NOME
    for (size_t i = 0; i < names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), names_[i]);
      if (it == msg->name.end()) {
        return;                        // manca un giunto: aspetta il prossimo messaggio
      }
      q_[i] = msg->position[std::distance(msg->name.begin(), it)];
    }
    initialized_ = true;
    RCLCPP_INFO(get_logger(), "Posizione iniziale letta da /joint_states: pronto.");
  }

  // Costruisce e invia un JointTrajectory con un solo punto
  void publishTarget(double duration)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = names_;         // stesso ordine di q_

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = q_;
    point.time_from_start = rclcpp::Duration::from_seconds(duration);
    traj.points.push_back(point);

    pub_->publish(traj);               // header.stamp = 0 -> "parti adesso"
  }

  const std::vector<std::string> names_{"j0", "j1", "j2", "j3"};
  std::vector<double> q_ = std::vector<double>(4, 0.0);       // obiettivo corrente

  // Limiti [rad]: j0 come da URDF (+-3.14); j1-j3 ridotti a +-1.57 per non
  // far ripiegare il braccio su se stesso o contro il tavolo
  const std::vector<double> lower_{-3.14, -1.57, -1.57, -1.57};
  const std::vector<double> upper_{3.14, 1.57, 1.57, 1.57};

  const double step_ = 0.05;           // passo per pressione [rad] (circa 3 gradi)
  const double move_time_ = 0.3;       // durata minima di un movimento [s]
  const double max_vel_ = 0.8;         // velocita' massima usata [rad/s] (URDF: 1.0)
  bool initialized_ = false;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

static void printUsage()
{
  std::printf(
    "\n=== Armando teleop ===\n"
    "  q / a : j0 +/-      w / s : j1 +/-\n"
    "  e / d : j2 +/-      r / f : j3 +/-\n"
    "  h     : home (tutti i giunti a 0)\n"
    "  x     : esci  (oppure Ctrl+C)\n\n");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmandoTeleop>();
  printUsage();

  RawTerminal terminal;
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);           // fa girare le callback (es. /joint_states)
    int key = terminal.readKey(50);    // aspetta un tasto per al massimo 50 ms
    if (key == -1) {
      continue;
    }
    if (key == 'x') {
      break;
    }
    if (!node->ready()) {
      RCLCPP_WARN(node->get_logger(), "In attesa di /joint_states...");
      continue;
    }
    node->handleKey(static_cast<char>(key));
  }

  rclcpp::shutdown();
  return 0;
}
