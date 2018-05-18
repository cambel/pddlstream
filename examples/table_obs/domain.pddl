(define (domain latent-table)
  (:requirements :strips :equality)
  (:predicates
    (Conf ?q)
    (Block ?b)
    (Pose ?b ?p)
    (Region ?r)
    (Traj ?t)
    (Kin ?b ?q ?p)
    (AtPose ?b ?p)
    (AtConf ?q)
    (Holding ?b)
    (HandEmpty)
    (CFree ?b1 ?p1 ?b2 ?p2)
    (PoseCollision ?b1 ?p1 ?b2 ?p2)
    (TrajCollision ?t ?b2 ?p2)
    (UnsafePose ?b ?p)
    (UnsafeTraj ?t)
    (CanMove)
    (Contained ?b ?p ?r)
    (In ?b ?r)
    (Placeable ?b ?r)
    (Motion ?q1 ?t ?q2)
  )
  (:functions
    (Distance ?q1 ?q2)
  )
  (:action move
    :parameters (?s1 ?s2)
    :precondition (and (Motion ?q1 ?t ?q2)
                       (AtConf ?q1) (CanMove) (not (UnsafeTraj ?t)))
    :effect (and (Holding ?q2)
                 (not (AtConf ?q1)) (not (CanMove)))
  )
  (:action pick
    :parameters (?a ?i ?s)
    :precondition (and (Arm ?a) (Stackable ?i ?s) (HandEmpty ?a)
                       (On ?i ?s) (Localized ?i) (Nearby ?s))
    :effect (and (Holding ?a ?i) (CanMove)
                 (not (On ?i ?s)) (not (HandEmpty ?a)))
  )

  (:derived (HoldingClass ?c)
    (exists (?a ?i) (and (Arm ?a) (Class ?i ?c)
                      (Holding ?a ?i))
  )

)