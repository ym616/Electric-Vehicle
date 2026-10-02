# Electric Vehicle C - Science Olympiad

This project contains the high-performance firmware and web dashboard for the Science Olympiad **Electric Vehicle (Division C)** event. It is designed to run on an **Arduino Uno R4 WiFi**.

## 🎯 Event Objective
The vehicle must travel a target distance, push a water bottle completely past a Bottle Line, and then reverse to stop exactly at the Target Point—all while finishing in exactly the Target Time. 

Scoring is based on:
1. **Distance Score**: How close the vehicle's Measurement Point is to the Target Point, plus how close the bottle is to the Bottle Line (0 if pushed past).
2. **Time Score**: How close the total run time is to the exact Target Time.
3. **Penalties/Bonuses**: e.g., Bottle completely beyond the line (-20 points).

## 🧠 Core Algorithms & Strategy

To get the absolute best (lowest) score possible, this firmware uses two major algorithms:

### 1. Initial Delay Time-Matching (Time Score)
The rules state: *"Any pause between the actuating the Vehicle and the Vehicle moving will be included in the Run Time."* 
Rather than trying to drive artificially slow to hit the 10-20 second Target Time, this code calculates a precise **Initial Delay**.
`Delay Time = Target Time - Estimated Travel Time`
When you press the start button with the pencil, the vehicle sits perfectly still, soaking up the exact amount of time needed. It then performs a high-speed, highly-repeatable sprint. This guarantees your Time Score is as close to 0.00 as possible.

### 2. Two-Stage Positional PID (Distance Score)
A standard PID (Proportional-Integral-Derivative) controller is used to drive the motors based on quadrature encoder feedback.
* **Stage 1 (Forward):** Setpoint = `Target Distance + Bottle Line Distance + Push Clearance`. The vehicle sprints forward, pushing the bottle securely over the line to guarantee the -20 point Bottle Bonus.
* **Stage 2 (Reverse):** Once settled forward, Setpoint = `Target Distance`. The PID automatically reverses the motor polarity and drives the vehicle backwards, parking it exactly on the Target Point for a perfect Distance Score.

## 🚀 Hardware Setup (Uno R4 WiFi)

* **Pin 9**: Motor Enable / PWM (ENA)
* **Pin 7**: Motor Direction 1 (IN1)
* **Pin 8**: Motor Direction 2 (IN2)
* **Pin 2**: Encoder Phase A (Interrupt)
* **Pin 3**: Encoder Phase B
* **Pin 4**: Start/Reset Microswitch (Connect to GND, uses internal pullup). This is what you actuate with the #2 pencil.

## 💻 How to Run & Calibrate

1. **Upload:** Open the project in VS Code with the PlatformIO extension. Connect your Arduino Uno R4 WiFi and click the **Upload** (→) button.
2. **Connect to Dashboard:** Connect your phone/laptop to the `Yash EV` WiFi network. Open `http://192.168.4.1` in your browser.
3. **Calibrate:**
   * Set your `cm/cnt` (Distance per encoder tick).
   * Do a few test runs to find your `Est. Travel Time` (how many seconds the physical movement takes).
4. **Compete:** On competition day, simply enter the Event Supervisor's `Target Distance`, `Bottle Line Distance`, and `Target Time` into the web dashboard, click **Apply Settings**, and actuate the pencil switch!
