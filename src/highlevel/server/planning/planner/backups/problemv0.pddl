(define (problem Planning_Problem_Simple)
    (:domain MDA_project) 
    (:objects 
      l0 l1 l2 l3 - Location 
      Tangy - Robot 
      Tiago Geoff - Person 
    )
    (:init 
      (connected l1 l0) 
      (connected l0 l1) 
      (connected l2 l1) 
      (connected l1 l2) 
      (connected l3 l2) 
      (connected l2 l3) 
      (at Tangy l0) 
      (isinroom l2 Geoff) 
      (isinroom l2 Tiago) 
    )
    (:goal 
        (at Tangy l2)
        ;;(observed l2)
    )
)