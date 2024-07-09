;; Auto-generated. Do not edit!


(when (boundp 'dynamic_biped::robotArmQVVD)
  (if (not (find-package "DYNAMIC_BIPED"))
    (make-package "DYNAMIC_BIPED"))
  (shadow 'robotArmQVVD (find-package "DYNAMIC_BIPED")))
(unless (find-package "DYNAMIC_BIPED::ROBOTARMQVVD")
  (make-package "DYNAMIC_BIPED::ROBOTARMQVVD"))

(in-package "ROS")
;;//! \htmlinclude robotArmQVVD.msg.html


(defclass dynamic_biped::robotArmQVVD
  :super ros::object
  :slots (_q _v _vd ))

(defmethod dynamic_biped::robotArmQVVD
  (:init
   (&key
    ((:q __q) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:v __v) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vd __vd) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _q __q)
   (setq _v __v)
   (setq _vd __vd)
   self)
  (:q
   (&optional __q)
   (if __q (setq _q __q)) _q)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:vd
   (&optional __vd)
   (if __vd (setq _vd __vd)) _vd)
  (:serialization-length
   ()
   (+
    ;; float64[] _q
    (* 8    (length _q)) 4
    ;; float64[] _v
    (* 8    (length _v)) 4
    ;; float64[] _vd
    (* 8    (length _vd)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _q
     (write-long (length _q) s)
     (dotimes (i (length _q))
       (sys::poke (elt _q i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _v
     (write-long (length _v) s)
     (dotimes (i (length _v))
       (sys::poke (elt _v i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _vd
     (write-long (length _vd) s)
     (dotimes (i (length _vd))
       (sys::poke (elt _vd i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _q
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _q (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _q i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _v
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _v (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _v i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _vd
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _vd (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _vd i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get dynamic_biped::robotArmQVVD :md5sum-) "f2840165d02c529a8a4e8e04370a219b")
(setf (get dynamic_biped::robotArmQVVD :datatype-) "dynamic_biped/robotArmQVVD")
(setf (get dynamic_biped::robotArmQVVD :definition-)
      "float64[] q
float64[] v
float64[] vd
")



(provide :dynamic_biped/robotArmQVVD "f2840165d02c529a8a4e8e04370a219b")


