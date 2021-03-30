;;Problem instance created by the Execution Module.

(define (problem Problem1-demo)
    (:domain TelepresenceDomainv14)
    (:objects 
      l0 l1 l2 l3 - Location 
      r1 - Robot 
      user1 user2 user3 - User 
      s1 s2 s3 - TelepresenceSession 
      cs1 - ChargingStation 
    )
    (:init 
      ;;Map topology
      (connected l0 l1) 
      (connected l1 l0) 
      (connected l1 l2) 
      (connected l2 l1) 
      (connected l2 l3) 
      (connected l3 l2) 
      (= (distance l0 l1) 20) 
      (= (distance l1 l0) 20) 
      (= (distance l1 l2) 20) 
      (= (distance l2 l1) 20) 
      (= (distance l2 l3) 20) 
      (= (distance l3 l2) 20) 
      (= (distance l0 l2) 20) 
      (= (distance l2 l0) 20) 
      
      ;;Charging stations
      (idle cs1) 
      (availableat cs1 l0) 
      (= (distancetostation l0 cs1) 0) 
      (= (distancetostation l1 cs1) 20) 
      (= (distancetostation l2 cs1) 60) 
      (= (distancetostation l3 cs1) 40) 
      
      ;;Users
      (at user1 l1) 
      (available user1) 
      (notinteracting user1) 
      (at user2 l2) 
      (available user2) 
      (notinteracting user2) 
      (at user3 l3) 
      (available user3) 
      (notinteracting user3) 
      
      ;;Robots
      ;; robot r1 
      (at r1 l0) 
      (ready r1) 
      (= (batterylevel r1) 17.5) 
      (= (batterycapacity r1) 19.1) 
      (= (velocity r1) 10.00) 
      (= (consumptionratio r1) 20.00) 
      (= (consumptionratiosession r1) 20.00) 
      (= (rechargeratio r1) 1.00) 
      
      ;;Telepresence Sessions
      (localuser s1 user1) 
      (sessionlocation s1 l1) 
      (= (sessionduration s1) 20) 
      (at 100.00 (mustbedone s1)) 
      (at 130.00 (not (mustbedone s1))) 
      (localuser s2 user2) 
      (sessionlocation s2 l2) 
      (= (sessionduration s2) 20) 
      (at 30.00 (mustbedone s2)) 
      (at 60.00 (not (mustbedone s2))) 
      (localuser s3 user3) 
      (sessionlocation s3 l3) 
      (= (sessionduration s3) 20) 
      (at 200.00 (mustbedone s3)) 
      (at 230.00 (not (mustbedone s3))) 
      
    )
    (:goal 
        ;;(at r1 l0)
        ;;(done s1)
        ;;(done s2)
        ;;(done s3)
        ;;(done s4)
        ;;(done s5)
	(at r1 l1)
    )
)