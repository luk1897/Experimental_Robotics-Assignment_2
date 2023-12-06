(define (problem prob)
(:domain motion)
(:objects
     r0 r1 r2 r3 r4 - region
     rob - robot
)
(:init
    (robot_in rob r0)
)

(:goal 
     (and 
          (visited r0)
          (visited r1)
          (visited r2)
          (visited r3)
          (visited r4)
     	  (robot_in rob r0)
     )
)
)


