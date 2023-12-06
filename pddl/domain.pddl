(define (domain motion)

(:requirements :typing :durative-actions :numeric-fluents :negative-preconditions :action-costs :conditional-effects :equality :fluents )


(:types 	robot region)


(:predicates
		(robot_in ?v - robot ?r - region)
		(free ?v - robot)
		(visited ?r - region)
	      
)


(:durative-action goto
		:parameters (?v - robot ?from ?to - region)
		:duration (= ?duration 100)
		:condition (and (at start (robot_in ?v ?from)))
	        :effect (and 
	        		(at start (not (robot_in ?v ?from))) 
				(at end (robot_in ?v ?to)) 
				(at end (visited ?to)))
)


)
