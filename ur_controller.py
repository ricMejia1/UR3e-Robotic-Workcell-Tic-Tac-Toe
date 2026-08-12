import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ur_msgs.srv import SetIO
from std_msgs.msg import String
import time

class UR3eActionController(Node):
    def __init__(self):
        super().__init__('ur_action_controller')

        # 1. Robot Configuration
        self.declare_parameter('robot_id', 1) 
        self.robot_id = self.get_parameter('robot_id').value

        # 2. Action Client for Movement
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        # 3. Service Client for all IO (Gripper and Conveyor)
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')

        # 4. Waypoints
        self.waypoint_names = [
            "home", "above_pallet", "approach_pallet", "pickup_pallet", "exit_pallet", 
            "above_lsensor", "approach_lsensor", "pickup_lsensor", "exit_lsensor", "above_grid",
            "approach_1", "approach_2", "approach_3", "approach_4", "approach_5", "approach_6", "approach_7", "approach_8", "approach_9", 
            "pickup_1", "pickup_2", "pickup_3", "pickup_4", "pickup_5", "pickup_6", "pickup_7", "pickup_8", "pickup_9", 
        ]
        for name in self.waypoint_names:
            self.declare_parameter(f'waypoints.{name}', [0.0]*6)

        self.block_color = None
        self.color_sub = self.create_subscription(String, 'block_color', self.color_callback, 10)

        self.get_logger().info(f"UR3e Controller for Robot {self.robot_id} Initialized")

    def color_callback(self,msg):
        self.block_color = msg.data

    def send_move_goal(self, waypoint_name):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return

        joint_positions = self.get_parameter(f'waypoints.{waypoint_name}').value
        goal_msg = FollowJointTrajectory.Goal()
        # Using your confirmed joint order
        goal_msg.trajectory.joint_names = [
        'shoulder_pan_joint', 
        'shoulder_lift_joint', 
        'elbow_joint', 
        'wrist_1_joint', 
        'wrist_2_joint', 
        'wrist_3_joint'
        ]

        # Log the start of movement
        self.get_logger().info(f"Moving to waypoint: {waypoint_name}")

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=1, nanosec=500000000) 
        goal_msg.trajectory.points.append(point)

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        # After the result_future is complete
        self.get_logger().info(f"Arrived at waypoint: {waypoint_name}")

    def pulse_gripper(self, state):
        """
        PULSE LOGIC:
        state 1 (Close): TO1 High, TO0 Low -> Wait -> Both Low
        state 0 (Open):  TO0 High, TO1 Low -> Wait -> Both Low
        """
        if not self.io_client.wait_for_service(timeout_sec=1.0):
            return

        # 1. TRIGGER THE MOVE
        req_pulse = SetIO.Request()
        req_pulse.fun = SetIO.Request.FUN_SET_DIGITAL_OUT

        action = "Closing" if state == 1 else "Opening"
        self.get_logger().info(f"Gripper: {action}...")

        if state == 1: # CLOSE
            # Set Close Pin High, Open Pin Low
            self._set_tool_io(SetIO.Request.PIN_TOOL_DOUT1, 1.0)
            self._set_tool_io(SetIO.Request.PIN_TOOL_DOUT0, 0.0)
        else: # OPEN
            # Set Open Pin High, Close Pin Low
            self._set_tool_io(SetIO.Request.PIN_TOOL_DOUT0, 1.0)
            self._set_tool_io(SetIO.Request.PIN_TOOL_DOUT1, 0.0)

        # 2. WAIT for the physical mechanism to move
        time.sleep(0.5)

        # 3. RESET BOTH TO 0 (The robot maintains position)
        self._set_tool_io(SetIO.Request.PIN_TOOL_DOUT0, 0.0)
        self._set_tool_io(SetIO.Request.PIN_TOOL_DOUT1, 0.0)

        self.get_logger().info(f"Gripper: {action} Complete (Pins Reset)")

    def _set_tool_io(self, pin, state):
        """Helper to send SetIO requests quickly"""
        req = SetIO.Request()
        req.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        req.pin = pin
        req.state = float(state)
        self.io_client.call_async(req)

    def power_conveyor(self, power):
        if not self.io_client.wait_for_service(timeout_sec=1.0):
            return
        
        status = f"Turning ON (Power: {power})" if power > 0 else "Turning OFF"
        self.get_logger().info(f"Conveyor: {status}")
        
        port = 0 if self.robot_id in [1, 2] else 1
        domain = 0 if self.robot_id in [1, 2] else 1
        # Apply a "Booster" for Robot 2 if power is requested
        if self.robot_id == 2 and power > 0:
            adjusted_power = min(1.0, power + 0.15) # Add 15% boost for stiction
        else:
            adjusted_power = power

        domain_req = SetIO.Request()
        domain_req.fun = 9
        domain_req.pin = port
        domain_req.state = float(domain)
        self.io_client.call_async(domain_req)

        power_req = SetIO.Request()
        power_req.fun = SetIO.Request.FUN_SET_ANALOG_OUT
        power_req.pin = port
        power_req.state = float(adjusted_power)
        self.io_client.call_async(power_req)

def print_board(board):
    print("\nCurrent board:")
    for i in range(0, 9, 3):
        print(" | ".join(board[i:i+3]))
    print()

def check_winner(board):
    win_patterns = [
        [0,1,2], [3,4,5], [6,7,8],   # Rows
        [0,3,6], [1,4,7], [2,5,8],   # Columns
        [0,4,8], [2,4,6]             # Diagonals
    ]
    for pattern in win_patterns:
        marks = [board[i] for i in pattern]
        if marks[0] != "-" and all(m == marks[0] for m in marks):
            return marks[0]
    return None

def robot_choose(board):
    win_patterns = [
        [0,1,2], [3,4,5], [6,7,8],
        [0,3,6], [1,4,7], [2,5,8],
        [0,4,8], [2,4,6]
    ]
    # Try to win
    for pattern in win_patterns:
        marks = [board[i] for i in pattern]
        if marks.count("O") == 2 and marks.count("-") == 1:
            return pattern[marks.index("-")]
    # Block user from winning
    for pattern in win_patterns:
        marks = [board[i] for i in pattern]
        if marks.count("X") == 2 and marks.count("-") == 1:
            return pattern[marks.index("-")]
    # Otherwise, take first open
    for i, v in enumerate(board):
        if v == "-":
            return i
    return None
        

def main():
    rclpy.init()
    node = UR3eActionController()

    board = ["-"] * 9

    try:
        # Wait 1 second
        time.sleep(1)
        
        # Open gripper and set Game Over variable to 0
        node.pulse_gripper(0)
        game_over = 0

        # Loop while game is still going on
        while game_over == 0:
            # Pick up block and place it in front of left sensor (above -> approach -> pickup -> exit -> above)
            node.send_move_goal("home")
            pick_sequence = ["above_pallet", "approach_pallet", "pickup_pallet"]
            for wp in pick_sequence: 
                node.send_move_goal(wp)
            node.pulse_gripper(1)
            time.sleep(1)
            pick_sequence = ["exit_pallet", "above_lsensor", "approach_lsensor", "pickup_lsensor"]
            for wp in pick_sequence: 
                node.send_move_goal(wp)
            node.pulse_gripper(0)
            time.sleep(1)
            node.send_move_goal("exit_lsensor")
            # Reset color and wait for a fresh sensor reading to ensure 
            # the turn logic uses the current block's data, not the previous one.
            node.block_color = None
            while node.block_color == None:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.send_move_goal("home")
            
            color = node.block_color.strip().upper()
            print(f"Detected color: {color}")

            
            # --- USER TURN (BROWN BLOCKS) ---
            # 1. Check if block color matches
            if color in ["BROWN", "WHITE"]:
            # 2. Return robot to home
                node.send_move_goal("home")
            # 3. Run conveyor belt until block falls off
                node.power_conveyor(0.8)
                time.sleep(3)
                node.power_conveyor(0.0)
            # 4. Notify the user it is their turn.
                print("USER TURN")
            # 5. Create a loop that stays active until a VALID move is entered.
                # 5a. Use try/except to catch non-integer inputs.
                #       Ex: 
                #           try: 
                #               user input portion
                #           except ValueError:
                #               print out error statement
                # 5b. Check that the number is 1-9 AND the board space is empty ("-").
                valid_move = False
                while not valid_move:
                    try:
                        pos = int(input("Enter the Space Number (1-9): ")) - 1

                        if pos < 0 or pos > 8:
                            print("Invalid move. Enter a number from 1 to 9.")
                        elif board[pos] != "-":
                            print("That space is already taken.")
                        else:
                            board[pos] = "X"
                            valid_move = True

                    except ValueError:
                        print("Invalid input. Enter an integer 1-9.")
            # 6. Once valid, update the board with "X" and print the new board state.


            # --- ROBOT TURN (RED, BLUE, GREEN) ---
            # 1. Check if the detected color matches a "Robot" block (RED, BLUE, or GREEN).
            elif color in ["RED", "BLUE", "GREEN"]:
                print("Robot turn!")

            # 2. Execute a sequence to pick up the block from the color sensor station.
                robot_pick_sequence = ["above_lsensor", "approach_lsensor", "pickup_lsensor"]
                for wp in robot_pick_sequence:
                    node.send_move_goal(wp)

                node.pulse_gripper(1)   # close gripper
                time.sleep(1)

                node.send_move_goal("exit_lsensor")
                node.send_move_goal("above_grid")

            # 3. Determine the robot's move:
            #    - Use the robot_choose(board) function to pick the best available square.
            #    - Note: This returns an index (0-8), so add 1 to match your waypoint names (1-9).
                pos = robot_choose(board)

                if pos is None:
                    print("No valid moves left for robot.")
                    game_over = 1
                    break

            # 4. Construct a movement sequence to the grid:
            #    - Hint: Use f-strings (f"approach_{variable}") to move to the chosen square.
                node.send_move_goal(f"approach_{pos + 1}")
                node.send_move_goal(f"pickup_{pos + 1}")

            # 5. Drop the block and return the robot to the 'home' position.
                node.pulse_gripper(0)   # open gripper
                time.sleep(1)

                node.send_move_goal("above_grid")
                node.send_move_goal("home")

                board[pos] = "O"

            # 6. Update the board array and print the board to the terminal.
            print_board(board)


            # --- STUDENT TASK: WIN CHECK & GAME OVER ---
            # 1. Use the check_winner(board) function to see if a player has won.
            winner = check_winner(board)
            # 2. If a winner exists:
            #    - Display the final board state.
            #    - Print a congratulatory message for the winner (X or O).
            #    - Set 'game_over' to 1 and 'break' the loop to end the game.
            if winner is not None:
                print(f"{winner} wins!")
                game_over = 1
                break
            # 3. Handle a Tie Game:
            #    - Check if there are no empty spaces ("-") left on the board.
            #    - If the board is full and no one has won, print "Tie game!" and 'break'.
            if "-" not in board:
                print("Tie game!")
                game_over = 1
                break
            

    except KeyboardInterrupt:
        pass
    finally:
        # Check if rclpy is still active before shutting down
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
