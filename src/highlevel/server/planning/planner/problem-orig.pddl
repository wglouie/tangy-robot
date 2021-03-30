;;Problem instance created by the Execution Module.

(define (problem Problem1-demo)
    (:domain MultiActivityDomainv4)
    (:objects 
      l0 l1 l2 l3 - Location
      cs1 - ChargingStation
      r1 - Robot
      user1 user2 user3 - User
      s1 s2 s3 - TelepresenceSession
      g1 g2 - BingoGame
    )
    (:init 
      ;;Global info
      ;;global: global 
      (= (distance l0 l1) 20)
      (= (distance l1 l0) 20)
      (= (distance l1 l2) 20)
      (= (distance l2 l1) 20)
      (= (distance l2 l3) 20)
      (= (distance l3 l2) 20)
      (= (distancetostation l0 cs1) 0)
      (= (distancetostation l1 cs1) 20)
      (= (distancetostation l2 cs1) 40)
      (= (distancetostation l3 cs1) 60)
      
      ;;Locations
      ;;location: l0 
      (connected l0 l1)
      
      ;;location: l1 
      (connected l1 l0)
      (connected l1 l2)
      
      ;;location: l2 
      (connected l2 l1)
      (connected l2 l3)
      
      ;;location: l3 
      (connected l3 l2)
      
      ;;Charging stations
      ;;station: cs1 
      (idle cs1)
      (availableat cs1 l0)
      
      ;;Users
      ;;user: user1 
      (at user1 l1)
      (available user1)
      (notinteracting user1)
      
      ;;user: user2 
      (at user2 l2)
      (available user2)
      (notinteracting user2)
      
      ;;user: user3 
      (at user3 l3)
      (available user3)
      (notinteracting user3)
      
      ;;Telepresence Sessions
      ;;session: s1 
      (localuser s1 user1)
      (sessionlocation s1 l1)
      (= (sessionduration s1) 20)
      (at 50.0 (mustbedone s1))
      (at 80.0 (not (mustbedone s1)))
      
      ;;session: s2 
      (localuser s2 user2)
      (sessionlocation s2 l2)
      (= (sessionduration s2) 20)
      (at 100.0 (mustbedone s2))
      (at 150.0 (not (mustbedone s2)))
      
      ;;session: s3 
      (localuser s3 user3)
      (sessionlocation s3 l1)
      (= (sessionduration s3) 20)
      (at 200.0 (mustbedone s3))
      (at 230.0 (not (mustbedone s3)))
      
      ;;Games
      ;;bingo: g1 
      (gamelocation g1 l0)
      (= (gameduration g1) 50)
      (mustbedone_game g1)
      (not_done_game g1)
      (need_reminder g1 user1)
      (need_reminder g1 user2)
      (need_reminder g1 user3)
      (= (reminder_duration g1) 10)
      (participant g1 user1)
      (participant g1 user2)
      (participant g1 user3)
      
      ;;bingo: g2 
      (gamelocation g2 l0)
      (= (gameduration g2) 50)
      (mustbedone_game g2)
      (not_done_game g2)
      (need_reminder g2 user1)
      (need_reminder g2 user2)
      (= (reminder_duration g2) 10)
      (participant g2 user1)
      (participant g2 user2)
      
      ;;Robots
      ;; robot r1 
      ;;robot: r1 
      (at r1 l0)
      (ready r1)
      (= (batterylevel r1) 1000)
      (= (batterycapacity r1) 1000)
      (= (velocity r1) 1.00)
      (= (consumptionratio r1) 20.00)
      (= (consumptionratiosession r1) 20.00)
      (= (rechargeratio r1) 1.00)
      
      
    )
    (:goal 
     (and        

        (done s1)
        (done s2)
        ;;(done s3)
        ;;(done s4)
        ;;(done s5)
	;;(at r1 l1)
	;;;(at r1 l0)

	(done_game g1)
      )
    )
)
