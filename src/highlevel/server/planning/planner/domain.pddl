(define (domain MultiActivityDomainv6)
    (:requirements :typing :fluents :negative-preconditions :disjunctive-preconditions :equality :quantified-preconditions :durative-actions :timed-initial-literals)
    (:types
      Location - object
      Mobile - object
      Robot - Mobile
      User - Mobile
      ChargingStation - object
      Activity - object
      TelepresenceSession - Activity
      BingoGame - Activity
    )
    (:predicates
      (connected ?loc - Location ?loc1 - Location)
      (at ?mob - Mobile ?loc - Location)
      (availableat ?cha - ChargingStation ?loc - Location)
      (participant ?bin - BingoGame ?use - User)
      (ready ?rob - Robot)
      (available ?use - User)
      (notinteracting ?use - User)
      (done ?tel - TelepresenceSession)
      (mustbedone ?tel - TelepresenceSession)
      (idle ?cha - ChargingStation)
      (mustbedone_game ?bin - BingoGame)
      (done_game ?bin - BingoGame)
      (not_done_game ?bin - BingoGame)
      (need_reminder ?bin - BingoGame ?u - User)
      (reminder_delivered ?bin - BingoGame ?u - User)
      (localuser ?tel - TelepresenceSession ?use - User)
      (sessionlocation ?tel - TelepresenceSession ?loc - Location)
      (gamelocation ?bin - BingoGame ?loc - Location)
    )
    (:functions
      (batterylevel ?rob - Robot)
      (batterycapacity ?rob - Robot)
      (velocity ?rob - Robot)
      (consumptionratio ?rob - Robot)
      (rechargeratio ?rob - Robot)
      (consumptionratiosession ?rob - Robot)
      (sessionduration ?tel - TelepresenceSession)
      (distance ?l1 - Location ?l2 - Location)
      (distancetostation ?loc - Location ?cs - ChargingStation)
      (gameduration ?bin - BingoGame)
      (reminder_duration ?bin - BingoGame)
    )
    (:durative-action move
     :parameters (?self - Robot ?from - Location ?to - Location ?cs - ChargingStation)
     :duration (= ?duration (/ (distance ?from ?to) (velocity ?self)))
     :condition 
       (and
         (at start (ready ?self))
         (at start (at ?self ?from))
         (at start (connected ?from ?to))
         (at start (not (= ?from ?to)))
         (at start (>= (batterylevel ?self) (/ (+ (distance ?from ?to) (distancetostation ?to ?cs)) (consumptionratio ?self))))
       )
     :effect
       (and
         (at start (not (at ?self ?from)))
         (at start (not (ready ?self)))
         (at end (at ?self ?to))
         (at end (ready ?self))
         (at start (decrease (batterylevel ?self) (/ (distance ?from ?to) (consumptionratio ?self))))
       )
    )

    (:durative-action dotelepresence
     :parameters (?self - Robot ?s - TelepresenceSession ?u - User ?loc - Location ?cs - ChargingStation)
     :duration (= ?duration (sessionduration ?s))
     :condition 
       (and
         (over all (at ?self ?loc))
         (over all (at ?u ?loc))
         (over all (available ?u))
         (over all (mustbedone ?s))
         (at start (ready ?self))
         (at start (at ?self ?loc))
         (at start (at ?u ?loc))
         (at start (available ?u))
         (at start (notinteracting ?u))
         (at start (localuser ?s ?u))
         (at start (sessionlocation ?s ?loc))
         (at start (mustbedone ?s))
         (at start (>= (batterylevel ?self) (+ (/ (sessionduration ?s) (consumptionratiosession ?self)) (/ (distancetostation ?loc ?cs) (consumptionratio ?self)))))
       )
     :effect
       (and
         (at start (not (ready ?self)))
         (at start (not (notinteracting ?u)))
         (at start (decrease (batterylevel ?self) (/ (sessionduration ?s) (consumptionratiosession ?self))))
         (at end (ready ?self))
         (at end (done ?s))
         (at end (notinteracting ?u))
         (at end (not (mustbedone ?s)))
       )
    )

    (:durative-action recharge
     :parameters (?self - Robot ?loc - Location ?cs - ChargingStation)
     :duration (= ?duration (* (- (batterycapacity ?self) (batterylevel ?self)) (rechargeratio ?self)))
     :condition 
       (and
         (at start (ready ?self))
         (at start (at ?self ?loc))
         (at start (availableat ?cs ?loc))
         (at start (idle ?cs))
         (over all (at ?self ?loc))
       )
     :effect
       (and
         (at start (not (idle ?cs)))
         (at start (not (ready ?self)))
         (at end (idle ?cs))
         (at end (ready ?self))
         (at start (assign (batterylevel ?self) (batterycapacity ?self)))
       )
    )

    (:durative-action playbingo
     :parameters (?self - Robot ?g - BingoGame ?loc - Location ?cs - ChargingStation)
     :duration (= ?duration (gameduration ?g))
     :condition 
       (and
         (at start (at ?self ?loc))
         (over all (at ?self ?loc))
         (at start (ready ?self))
         (at start (mustbedone_game ?g))
         (over all (mustbedone_game ?g))
         (at start (gamelocation ?g ?loc))
         (at start (>= (batterylevel ?self) (+ (/ (gameduration ?g) (consumptionratiosession ?self)) (/ (distancetostation ?loc ?cs) (consumptionratio ?self)))))
         (at start (not_done_game ?g))
         (at start (forall (?p - User)
	   (imply
	     (participant ?g ?p)
	     (reminder_delivered ?g ?p)
	   )
	 ))
         (at start (forall (?p - User)
	   (imply
	     (participant ?g ?p)
	     (available ?p)
	   )
	 ))
         (over all (forall (?p - User)
	   (imply
	     (participant ?g ?p)
	     (available ?p)
	   )
	 ))
       )
     :effect
       (and
         (at start (not (ready ?self)))
         (at start (decrease (batterylevel ?self) (/ (gameduration ?g) (consumptionratiosession ?self))))
         (at end (ready ?self))
         (at end (done_game ?g))
         (at end (not (mustbedone_game ?g)))
         (at start (not (not_done_game ?g)))
       )
    )

    (:durative-action remind
     :parameters (?self - Robot ?u - User ?g - BingoGame ?loc - Location ?cs - ChargingStation)
     :duration (= ?duration (reminder_duration ?g))
     :condition 
       (and
         (over all (at ?self ?loc))
         (over all (at ?u ?loc))
         (over all (available ?u))
         (at start (ready ?self))
         (at start (at ?self ?loc))
         (at start (at ?u ?loc))
         (at start (available ?u))
         (at start (notinteracting ?u))
         (at start (>= (batterylevel ?self) (+ (/ (reminder_duration ?g) (consumptionratiosession ?self)) (/ (distancetostation ?loc ?cs) (consumptionratio ?self)))))
         (at start (participant ?g ?u))
         (at start (not_done_game ?g))
         (at start (need_reminder ?g ?u))
       )
     :effect
       (and
         (at start (not (ready ?self)))
         (at start (not (notinteracting ?u)))
         (at start (decrease (batterylevel ?self) (/ 5 (consumptionratiosession ?self))))
         (at end (ready ?self))
         (at end (notinteracting ?u))
         (at start (not (need_reminder ?g ?u)))
         (at start (reminder_delivered ?g ?u))
       )
    )

)

