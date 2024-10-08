# earthrovers_description
This packages houses a rough geometric URDF description of the FrodoBot Earth Rover.

## `frodobot.urdf`
URDF description of the FrodoBot based on the provided [hardware specification document](https://github.com/frodobots-org/earth-rovers-sdk?tab=readme-ov-file#hardware-specs).

### Notes/Disclaimers
- This URDF is not *completely* accurate / 100% consistent with the hardware
  spec document. There were some URDF/Xacro concepts that we didn't have time to
  investigate further, and as a result, the wheels (for example) might not be in
  the *exactly* correct position--but may be close enough for all intents and
  purposes. We also didn't have time to add meshes to make the visualization
  pretty. **If you can contribute to improving the description in any way,
  please feel free to contribute/open an issue!**