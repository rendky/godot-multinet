extends Node

@onready var multinet_node = $MultinetBCCMNode3D
@onready var camera = $Camera3D

var test_failed = false
var max_resolved_layer_global = 0

enum Phase {
	WARM_SOURCE,
	APPROACH_EDGE,
	WARM_OVERLAP,
	CROSS_EDGE,
	WARM_DESTINATION,
	VALIDATE
}

var current_phase: int = Phase.WARM_SOURCE
var phase_elapsed: float = 0.0
var signed_distance: float = 512.0
var frame_epoch: int = 1
var test_finished: bool = false

func _ready():
	print("===================================================")
	print("  MULTINET WP5 RUNTIME EDGE-CROSSING GATE ")
	print("===================================================")
	print("[INFO] Starting state-driven canonical edge crossing...")
	
	var forward := Vector3(1.0, -0.08, 0.2).normalized()
	camera.look_at(camera.global_position + forward, Vector3.UP)

func assert_cond(condition, message):
	if condition:
		print("[PASS] " + message)
	else:
		print("[FAIL] " + message)
		test_failed = true

func transition_to(new_phase: int):
	current_phase = new_phase
	phase_elapsed = 0.0

func _process(delta: float):
	if test_finished:
		return
		
	phase_elapsed += delta
	
	match current_phase:
		Phase.WARM_SOURCE:
			signed_distance = 512.0
			frame_epoch = 1
			
			if phase_elapsed > 10.0:
				assert_cond(false, "Timeout waiting for WarmSource conditions")
				transition_to(Phase.VALIDATE)
			else:
				var summary = multinet_node.get_debug_summary()
				var vis_src = summary.get("visible source-face block count", 0)
				var nf_src = summary.get("non-fallback source-face page count", 0)
				var fallbacks = summary.get("fallback_slot_valid_count", 0)
				
				if vis_src > 0 and nf_src > 0 and fallbacks == 8:
					print("[INFO] WARM_SOURCE complete after ", phase_elapsed, "s")
					assert_cond(true, "Source face resident")
					transition_to(Phase.APPROACH_EDGE)

		Phase.APPROACH_EDGE:
			# Move -50m per second
			signed_distance -= 50.0 * delta
			frame_epoch = 1
			
			if signed_distance <= 32.0:
				signed_distance = 32.0
				print("[INFO] APPROACH_EDGE complete")
				transition_to(Phase.WARM_OVERLAP)
				
		Phase.WARM_OVERLAP:
			signed_distance = 32.0
			frame_epoch = 1
			
			if phase_elapsed > 10.0:
				assert_cond(false, "Timeout waiting for WarmOverlap conditions")
				transition_to(Phase.VALIDATE)
			else:
				var summary = multinet_node.get_debug_summary()
				var vis_dest = summary.get("visible destination-face block count", 0)
				var nf_src = summary.get("non-fallback source-face page count", 0)
				var nf_dest = summary.get("non-fallback destination-face page count", 0)
				
				if vis_dest > 0 and nf_src > 0 and nf_dest > 0:
					print("[INFO] WARM_OVERLAP complete after ", phase_elapsed, "s")
					assert_cond(true, "Overlap footprint resident")
					transition_to(Phase.CROSS_EDGE)
					
		Phase.CROSS_EDGE:
			signed_distance = -32.0
			frame_epoch = 2
			print("[INFO] CROSS_EDGE executed")
			transition_to(Phase.WARM_DESTINATION)
			
		Phase.WARM_DESTINATION:
			signed_distance = -32.0
			frame_epoch = 2
			
			if phase_elapsed > 10.0:
				assert_cond(false, "Timeout waiting for WarmDestination conditions")
				transition_to(Phase.VALIDATE)
			else:
				var summary = multinet_node.get_debug_summary()
				var vis_src = summary.get("visible source-face block count", 0) # After crossing, active face is swapped!
				var nf_src = summary.get("non-fallback source-face page count", 0)
				var max_layer = summary.get("maximum submitted texture layer", 0)
				var fallbacks = summary.get("fallback_slot_valid_count", 0)
				
				var lods_active = true
				for lod in range(8):
					if summary.get("lod_%d_visible" % lod, 0) == 0:
						lods_active = false
						
				if int(phase_elapsed * 10) % 10 == 0:
					print("[DEBUG] WARM_DESTINATION vis_src=", vis_src, " nf_src=", nf_src, " max_layer=", max_layer, " fallbacks=", fallbacks, " lods_active=", lods_active)
					for lod in range(8):
						print("  lod_", lod, "_visible=", summary.get("lod_%d_visible" % lod, 0))
						
				if vis_src > 0 and nf_src > 0 and max_layer > 0 and fallbacks == 8 and lods_active:
					print("[INFO] WARM_DESTINATION complete after ", phase_elapsed, "s")
					assert_cond(true, "Destination footprint resident")
					transition_to(Phase.VALIDATE)

		Phase.VALIDATE:
			print("[INFO] Validating continuous traversal invariants...")
			assert_cond(max_resolved_layer_global <= 127, "All submitted layers are within 0..127")
			
			if not test_failed:
				print("===================================================")
				print("  BCCM-SURFACE-EDGE-01: PASSED ")
				print("===================================================")
			else:
				print("===================================================")
				print("  BCCM-SURFACE-EDGE-01: FAILED ")
				print("===================================================")
				
			test_finished = true
			get_tree().quit(1 if test_failed else 0)

	if not test_finished:
		multinet_node.debug_publish_synthetic_edge_camera(camera, signed_distance, frame_epoch)
		
		var summary = multinet_node.get_debug_summary()
		max_resolved_layer_global = max(max_resolved_layer_global, summary.get("maximum submitted texture layer", 0))
