;; Auto-generated. Do not edit!


(when (boundp 'unitree_legged_msgs::HighState)
  (if (not (find-package "UNITREE_LEGGED_MSGS"))
    (make-package "UNITREE_LEGGED_MSGS"))
  (shadow 'HighState (find-package "UNITREE_LEGGED_MSGS")))
(unless (find-package "UNITREE_LEGGED_MSGS::HIGHSTATE")
  (make-package "UNITREE_LEGGED_MSGS::HIGHSTATE"))

(in-package "ROS")
;;//! \htmlinclude HighState.msg.html


(defclass unitree_legged_msgs::HighState
  :super ros::object
  :slots (_levelFlag _commVersion _robotID _SN _bandWidth _mode _progress _imu _gaitType _footRaiseHeight _position _bodyHeight _velocity _yawSpeed _footPosition2Body _footSpeed2Body _bms _footForce _footForceEst _wirelessRemote _reserve _crc ))

(defmethod unitree_legged_msgs::HighState
  (:init
   (&key
    ((:levelFlag __levelFlag) 0)
    ((:commVersion __commVersion) 0)
    ((:robotID __robotID) 0)
    ((:SN __SN) 0)
    ((:bandWidth __bandWidth) 0)
    ((:mode __mode) 0)
    ((:progress __progress) 0.0)
    ((:imu __imu) (instance unitree_legged_msgs::IMU :init))
    ((:gaitType __gaitType) 0)
    ((:footRaiseHeight __footRaiseHeight) 0.0)
    ((:position __position) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:bodyHeight __bodyHeight) 0.0)
    ((:velocity __velocity) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:yawSpeed __yawSpeed) 0.0)
    ((:footPosition2Body __footPosition2Body) (let (r) (dotimes (i 4) (push (instance unitree_legged_msgs::Cartesian :init) r)) r))
    ((:footSpeed2Body __footSpeed2Body) (let (r) (dotimes (i 4) (push (instance unitree_legged_msgs::Cartesian :init) r)) r))
    ((:bms __bms) (instance unitree_legged_msgs::BmsState :init))
    ((:footForce __footForce) (make-array 4 :initial-element 0 :element-type :integer))
    ((:footForceEst __footForceEst) (make-array 4 :initial-element 0 :element-type :integer))
    ((:wirelessRemote __wirelessRemote) (make-array 40 :initial-element 0 :element-type :char))
    ((:reserve __reserve) 0)
    ((:crc __crc) 0)
    )
   (send-super :init)
   (setq _levelFlag (round __levelFlag))
   (setq _commVersion (round __commVersion))
   (setq _robotID (round __robotID))
   (setq _SN (round __SN))
   (setq _bandWidth (round __bandWidth))
   (setq _mode (round __mode))
   (setq _progress (float __progress))
   (setq _imu __imu)
   (setq _gaitType (round __gaitType))
   (setq _footRaiseHeight (float __footRaiseHeight))
   (setq _position __position)
   (setq _bodyHeight (float __bodyHeight))
   (setq _velocity __velocity)
   (setq _yawSpeed (float __yawSpeed))
   (setq _footPosition2Body __footPosition2Body)
   (setq _footSpeed2Body __footSpeed2Body)
   (setq _bms __bms)
   (setq _footForce __footForce)
   (setq _footForceEst __footForceEst)
   (setq _wirelessRemote __wirelessRemote)
   (setq _reserve (round __reserve))
   (setq _crc (round __crc))
   self)
  (:levelFlag
   (&optional __levelFlag)
   (if __levelFlag (setq _levelFlag __levelFlag)) _levelFlag)
  (:commVersion
   (&optional __commVersion)
   (if __commVersion (setq _commVersion __commVersion)) _commVersion)
  (:robotID
   (&optional __robotID)
   (if __robotID (setq _robotID __robotID)) _robotID)
  (:SN
   (&optional __SN)
   (if __SN (setq _SN __SN)) _SN)
  (:bandWidth
   (&optional __bandWidth)
   (if __bandWidth (setq _bandWidth __bandWidth)) _bandWidth)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:progress
   (&optional __progress)
   (if __progress (setq _progress __progress)) _progress)
  (:imu
   (&rest __imu)
   (if (keywordp (car __imu))
       (send* _imu __imu)
     (progn
       (if __imu (setq _imu (car __imu)))
       _imu)))
  (:gaitType
   (&optional __gaitType)
   (if __gaitType (setq _gaitType __gaitType)) _gaitType)
  (:footRaiseHeight
   (&optional __footRaiseHeight)
   (if __footRaiseHeight (setq _footRaiseHeight __footRaiseHeight)) _footRaiseHeight)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:bodyHeight
   (&optional __bodyHeight)
   (if __bodyHeight (setq _bodyHeight __bodyHeight)) _bodyHeight)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:yawSpeed
   (&optional __yawSpeed)
   (if __yawSpeed (setq _yawSpeed __yawSpeed)) _yawSpeed)
  (:footPosition2Body
   (&rest __footPosition2Body)
   (if (keywordp (car __footPosition2Body))
       (send* _footPosition2Body __footPosition2Body)
     (progn
       (if __footPosition2Body (setq _footPosition2Body (car __footPosition2Body)))
       _footPosition2Body)))
  (:footSpeed2Body
   (&rest __footSpeed2Body)
   (if (keywordp (car __footSpeed2Body))
       (send* _footSpeed2Body __footSpeed2Body)
     (progn
       (if __footSpeed2Body (setq _footSpeed2Body (car __footSpeed2Body)))
       _footSpeed2Body)))
  (:bms
   (&rest __bms)
   (if (keywordp (car __bms))
       (send* _bms __bms)
     (progn
       (if __bms (setq _bms (car __bms)))
       _bms)))
  (:footForce
   (&optional __footForce)
   (if __footForce (setq _footForce __footForce)) _footForce)
  (:footForceEst
   (&optional __footForceEst)
   (if __footForceEst (setq _footForceEst __footForceEst)) _footForceEst)
  (:wirelessRemote
   (&optional __wirelessRemote)
   (if __wirelessRemote (setq _wirelessRemote __wirelessRemote)) _wirelessRemote)
  (:reserve
   (&optional __reserve)
   (if __reserve (setq _reserve __reserve)) _reserve)
  (:crc
   (&optional __crc)
   (if __crc (setq _crc __crc)) _crc)
  (:serialization-length
   ()
   (+
    ;; uint8 _levelFlag
    1
    ;; uint16 _commVersion
    2
    ;; uint16 _robotID
    2
    ;; uint32 _SN
    4
    ;; uint8 _bandWidth
    1
    ;; uint8 _mode
    1
    ;; float32 _progress
    4
    ;; unitree_legged_msgs/IMU _imu
    (send _imu :serialization-length)
    ;; uint8 _gaitType
    1
    ;; float32 _footRaiseHeight
    4
    ;; float32[3] _position
    (* 4    3)
    ;; float32 _bodyHeight
    4
    ;; float32[3] _velocity
    (* 4    3)
    ;; float32 _yawSpeed
    4
    ;; unitree_legged_msgs/Cartesian[4] _footPosition2Body
    (apply #'+ (send-all _footPosition2Body :serialization-length))
    ;; unitree_legged_msgs/Cartesian[4] _footSpeed2Body
    (apply #'+ (send-all _footSpeed2Body :serialization-length))
    ;; unitree_legged_msgs/BmsState _bms
    (send _bms :serialization-length)
    ;; int16[4] _footForce
    (* 2    4)
    ;; int16[4] _footForceEst
    (* 2    4)
    ;; uint8[40] _wirelessRemote
    (* 1    40)
    ;; uint32 _reserve
    4
    ;; uint32 _crc
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _levelFlag
       (write-byte _levelFlag s)
     ;; uint16 _commVersion
       (write-word _commVersion s)
     ;; uint16 _robotID
       (write-word _robotID s)
     ;; uint32 _SN
       (write-long _SN s)
     ;; uint8 _bandWidth
       (write-byte _bandWidth s)
     ;; uint8 _mode
       (write-byte _mode s)
     ;; float32 _progress
       (sys::poke _progress (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; unitree_legged_msgs/IMU _imu
       (send _imu :serialize s)
     ;; uint8 _gaitType
       (write-byte _gaitType s)
     ;; float32 _footRaiseHeight
       (sys::poke _footRaiseHeight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[3] _position
     (dotimes (i 3)
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _bodyHeight
       (sys::poke _bodyHeight (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[3] _velocity
     (dotimes (i 3)
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _yawSpeed
       (sys::poke _yawSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; unitree_legged_msgs/Cartesian[4] _footPosition2Body
     (dolist (elem _footPosition2Body)
       (send elem :serialize s)
       )
     ;; unitree_legged_msgs/Cartesian[4] _footSpeed2Body
     (dolist (elem _footSpeed2Body)
       (send elem :serialize s)
       )
     ;; unitree_legged_msgs/BmsState _bms
       (send _bms :serialize s)
     ;; int16[4] _footForce
     (dotimes (i 4)
       (write-word (elt _footForce i) s)
       )
     ;; int16[4] _footForceEst
     (dotimes (i 4)
       (write-word (elt _footForceEst i) s)
       )
     ;; uint8[40] _wirelessRemote
     (princ _wirelessRemote s)
     ;; uint32 _reserve
       (write-long _reserve s)
     ;; uint32 _crc
       (write-long _crc s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _levelFlag
     (setq _levelFlag (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _commVersion
     (setq _commVersion (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _robotID
     (setq _robotID (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _SN
     (setq _SN (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _bandWidth
     (setq _bandWidth (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _progress
     (setq _progress (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; unitree_legged_msgs/IMU _imu
     (send _imu :deserialize buf ptr-) (incf ptr- (send _imu :serialization-length))
   ;; uint8 _gaitType
     (setq _gaitType (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _footRaiseHeight
     (setq _footRaiseHeight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[3] _position
   (dotimes (i (length _position))
     (setf (elt _position i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _bodyHeight
     (setq _bodyHeight (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[3] _velocity
   (dotimes (i (length _velocity))
     (setf (elt _velocity i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32 _yawSpeed
     (setq _yawSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; unitree_legged_msgs/Cartesian[4] _footPosition2Body
   (dotimes (i 4)
     (send (elt _footPosition2Body i) :deserialize buf ptr-) (incf ptr- (send (elt _footPosition2Body i) :serialization-length))
     )
   ;; unitree_legged_msgs/Cartesian[4] _footSpeed2Body
   (dotimes (i 4)
     (send (elt _footSpeed2Body i) :deserialize buf ptr-) (incf ptr- (send (elt _footSpeed2Body i) :serialization-length))
     )
   ;; unitree_legged_msgs/BmsState _bms
     (send _bms :deserialize buf ptr-) (incf ptr- (send _bms :serialization-length))
   ;; int16[4] _footForce
   (dotimes (i (length _footForce))
     (setf (elt _footForce i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; int16[4] _footForceEst
   (dotimes (i (length _footForceEst))
     (setf (elt _footForceEst i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; uint8[40] _wirelessRemote
   (setq _wirelessRemote (make-array 40 :element-type :char))
   (replace _wirelessRemote buf :start2 ptr-) (incf ptr- 40)
   ;; uint32 _reserve
     (setq _reserve (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _crc
     (setq _crc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get unitree_legged_msgs::HighState :md5sum-) "b52a86799f89d67cea890e18e2a7bf68")
(setf (get unitree_legged_msgs::HighState :datatype-) "unitree_legged_msgs/HighState")
(setf (get unitree_legged_msgs::HighState :definition-)
      "uint8 levelFlag
uint16 commVersion                  # Old version Aliengo does not have
uint16 robotID                      # Old version Aliengo does not have
uint32 SN                           # Old version Aliengo does not have
uint8 bandWidth                     # Old version Aliengo does not have
uint8 mode
float32 progress                    # new on Go1, reserve
IMU imu
uint8 gaitType                      # new on Go1, 0.idle  1.trot  2.trot running  3.climb stair
float32 footRaiseHeight             # (unit: m, default: 0.08m), foot up height while walking
float32[3] position                 # (unit: m), from own odometry in inertial frame, usually drift
float32 bodyHeight                  # (unit: m, default: 0.28m)
float32[3] velocity                 # (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
float32 yawSpeed                    # (unit: rad/s), rotateSpeed in body frame        
Cartesian[4] footPosition2Body      # foot position relative to body
Cartesian[4] footSpeed2Body         # foot speed relative to body
#int8[20] temperature
BmsState bms
int16[4] footForce                  # Old version Aliengo is different
int16[4] footForceEst               # Old version Aliengo does not have
uint8[40] wirelessRemote
uint32 reserve                      # Old version Aliengo does not have
uint32 crc
================================================================================
MSG: unitree_legged_msgs/IMU
float32[4] quaternion
float32[3] gyroscope
float32[3] accelerometer
int8 temperature
================================================================================
MSG: unitree_legged_msgs/Cartesian
float32 x
float32 y
float32 z
================================================================================
MSG: unitree_legged_msgs/BmsState
uint8 version_h
uint8 version_l
uint8 bms_status
uint8 SOC                  # SOC 0-100%
int32 current              # mA
uint16 cycle
int8[2] BQ_NTC             # x1 degrees centigrade
int8[2] MCU_NTC            # x1 degrees centigrade
uint16[10] cell_vol        # cell voltage mV
")



(provide :unitree_legged_msgs/HighState "b52a86799f89d67cea890e18e2a7bf68")


