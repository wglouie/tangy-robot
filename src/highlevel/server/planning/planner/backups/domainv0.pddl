(define (domain MDA_project)
    (:requirements :typing :quantified-preconditions :conditional-effects)
    (:types
      Location - object
      Person - object
      Mobile - object
      Robot - Mobile
    )
    (:predicates
      (connected ?loc - Location ?loc1 - Location)
      (at ?mob - Mobile ?loc - Location)
      (isinroom ?loc - Location ?per - Person)
      (scanned ?loc - Location)
      (observed ?loc - Location)
      (locationknown ?per - Person)
      (lastseenat ?per - Person ?loc - Location)
    )
    (:action move
     :parameters (?self - Robot ?from - Location ?to - Location)
     :precondition 
       (and
         (at ?self ?from)
         (connected ?from ?to)
       )
     :effect
       (and
         (at ?self ?to)
         (not (at ?self ?from))
       )
    )

    (:action identify
     :parameters (?self - Robot ?loc - Location)
     :precondition 
       (at ?self ?loc)
     :effect
       (and
         (scanned ?loc)
         (forall (?p - Person)
           (when
             (isinroom ?loc ?p)
             (and
               (lastseenat ?p ?loc)
               (locationknown ?p)
             )
           )
         )
       )
    )

    (:action turn
     :parameters (?self - Robot ?loc - Location)
     :precondition 
       (at ?self ?loc)
     :effect
       (observed ?loc)
    )

)
