;; Auto-generated. Do not edit!


(when (boundp 'FP_Magang::koordinat)
  (if (not (find-package "FP_MAGANG"))
    (make-package "FP_MAGANG"))
  (shadow 'koordinat (find-package "FP_MAGANG")))
(unless (find-package "FP_MAGANG::KOORDINAT")
  (make-package "FP_MAGANG::KOORDINAT"))

(in-package "ROS")
;;//! \htmlinclude koordinat.msg.html


(defclass FP_Magang::koordinat
  :super ros::object
  :slots (_x _y _z ))

(defmethod FP_Magang::koordinat
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get FP_Magang::koordinat :md5sum-) "cc153912f1453b708d221682bc23d9ac")
(setf (get FP_Magang::koordinat :datatype-) "FP_Magang/koordinat")
(setf (get FP_Magang::koordinat :definition-)
      "float32 x
float32 y
float32 z
")



(provide :FP_Magang/koordinat "cc153912f1453b708d221682bc23d9ac")


