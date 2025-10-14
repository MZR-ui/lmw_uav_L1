;; Auto-generated. Do not edit!


(when (boundp 'gimbal_control_serial::GimbalCmd)
  (if (not (find-package "GIMBAL_CONTROL_SERIAL"))
    (make-package "GIMBAL_CONTROL_SERIAL"))
  (shadow 'GimbalCmd (find-package "GIMBAL_CONTROL_SERIAL")))
(unless (find-package "GIMBAL_CONTROL_SERIAL::GIMBALCMD")
  (make-package "GIMBAL_CONTROL_SERIAL::GIMBALCMD"))

(in-package "ROS")
;;//! \htmlinclude GimbalCmd.msg.html


(defclass gimbal_control_serial::GimbalCmd
  :super ros::object
  :slots (_roll _pitch _yaw _mode ))

(defmethod gimbal_control_serial::GimbalCmd
  (:init
   (&key
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _mode (round __mode))
   self)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; float32 _roll
    4
    ;; float32 _pitch
    4
    ;; float32 _yaw
    4
    ;; uint8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get gimbal_control_serial::GimbalCmd :md5sum-) "95e52dfca23ef72d66122799a9e1eabc")
(setf (get gimbal_control_serial::GimbalCmd :datatype-) "gimbal_control_serial/GimbalCmd")
(setf (get gimbal_control_serial::GimbalCmd :definition-)
      "float32 roll
float32 pitch
float32 yaw
uint8 mode

")



(provide :gimbal_control_serial/GimbalCmd "95e52dfca23ef72d66122799a9e1eabc")


