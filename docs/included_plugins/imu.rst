.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

IMU
===
This plugin simulates a 2D inertial measurement unit mounted on a body, and
publishes `sensor_msgs/Imu <https://docs.ros.org/en/rolling/p/sensor_msgs/interfaces/msg/Imu.html>`_
messages in its own frame:

* orientation: the yaw of the IMU frame in the world (roll and pitch are 0)
* angular velocity: the body's yaw rate, on z
* linear acceleration: of the IMU point, from the velocity the physics solver
  produced at consecutive steps, so a turn reads as centripetal acceleration
  and a collision as a spike, plus gravity (9.80665 m/s^2) on z

Each reading can carry gaussian noise; the message's covariances then give
its variance. A teleport (``move_model``) that changes the heading of a moving
body reads as one large acceleration spike, as the velocity turns with it.

.. code-block:: yaml

  plugins:

      # required, specify Imu type to load the plugin
    - type: Imu

      # required, name of the plugin
      name: imu

      # required, body the IMU is mounted on
      body: base

      # optional, defaults to "imu", the topic to publish on
      topic: imu

      # optional, default to name of this plugin, the frame id of the messages
      # and of the TF published with broadcast_tf
      frame: imu_link

      # optional, defaults to true, whether to broadcast the body -> IMU transform
      broadcast_tf: true

      # optional, defaults to inf, rate to publish at, in Hz. The reading itself
      # updates every physics step
      update_rate: .inf

      # optional, default to [0, 0, 0], in the form of [x, y, yaw], the position
      # and orientation of the IMU frame relative to the body
      origin: [0, 0, 0]

      # optional, default to 0 (no noise), >= 0: standard deviations of the
      # gaussian noise on the orientation (rad), angular velocity (rad/s) and
      # linear acceleration (m/s^2)
      orientation_noise_std_dev: 0.0
      angular_velocity_noise_std_dev: 0.0
      linear_acceleration_noise_std_dev: 0.0

      # optional, defaults to 0 (a random seed each run): seeds the noise
      seed: 0
