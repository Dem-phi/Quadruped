;; Auto-generated. Do not edit!


(when (boundp 'wtr_serial::actor_1)
  (if (not (find-package "WTR_SERIAL"))
    (make-package "WTR_SERIAL"))
  (shadow 'actor_1 (find-package "WTR_SERIAL")))
(unless (find-package "WTR_SERIAL::ACTOR_1")
  (make-package "WTR_SERIAL::ACTOR_1"))

(in-package "ROS")
;;//! \htmlinclude actor_1.msg.html


(defclass wtr_serial::actor_1
  :super ros::object
  :slots (_id _agl_0 _agl_1 _agl_2 ))

(defmethod wtr_serial::actor_1
  (:init
   (&key
    ((:id __id) 0)
    ((:agl_0 __agl_0) 0.0)
    ((:agl_1 __agl_1) 0.0)
    ((:agl_2 __agl_2) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _agl_0 (float __agl_0))
   (setq _agl_1 (float __agl_1))
   (setq _agl_2 (float __agl_2))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:agl_0
   (&optional __agl_0)
   (if __agl_0 (setq _agl_0 __agl_0)) _agl_0)
  (:agl_1
   (&optional __agl_1)
   (if __agl_1 (setq _agl_1 __agl_1)) _agl_1)
  (:agl_2
   (&optional __agl_2)
   (if __agl_2 (setq _agl_2 __agl_2)) _agl_2)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; float32 _agl_0
    4
    ;; float32 _agl_1
    4
    ;; float32 _agl_2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; float32 _agl_0
       (sys::poke _agl_0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _agl_1
       (sys::poke _agl_1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _agl_2
       (sys::poke _agl_2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _agl_0
     (setq _agl_0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _agl_1
     (setq _agl_1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _agl_2
     (setq _agl_2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get wtr_serial::actor_1 :md5sum-) "41170c4af0e4f57d0ef889762422e6e9")
(setf (get wtr_serial::actor_1 :datatype-) "wtr_serial/actor_1")
(setf (get wtr_serial::actor_1 :definition-)
      "uint8 id
float32 agl_0
float32 agl_1
float32 agl_2
")



(provide :wtr_serial/actor_1 "41170c4af0e4f57d0ef889762422e6e9")


