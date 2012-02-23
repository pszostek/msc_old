# Always in context:
#
# r4 : Physical address to status registers
# r5 : Physical address to data region

#
# Both r4 and r5 must be DW aligned.
# Note that prefetching is allowed, so up to 8 useless DWs may be read.

# First, load the status registers into SDMA space
start:
	stf r4, 0x20   # To MSA, prefetch on, address is incremental
	ldf r7, 0x2b   # r7 = status register
	btsti r7, 4    # Check bit 4 (REQ valid)

	bt cont1       # Bit 4 == 1 means there's a request

	# If we got here, we're done
	done 4         # Quit EVENT
	bf start
	
cont1:					
	ldf r1, 0x2b # r1 = source/dest address
	ldf r2, 0x2b # r2 = Number of DWs (may be zero on write op)
	ldf r6, 0x2b # r6 = Number of DWs to 32-byte boundary 	

	btsti r7, 2    # Check bit 2 (Read/Write)
	bt write       # Bit 2 == 1 means Write
	
	# If we got here, we're expected to read

	# Prefetch is unlikely to contribute to performace here, as the bulk
	# operations are done in successive copies, so the processor is
	# likely to wait on the next copy command anyhow.
	
	stf r1, 0x00 # To MSA, NO prefetch, address is incremental

readloop:
	stf r5, 0x04 # To MDA, address is incremental
	cmphs r6, r2 # Is r6 larger or equal to the number of DWs left to copy?
	bt lastread  # If so, jump to last transfer label
	stf r6, 0x18 # Copy r6 words from MSA to MDA address.
	yield
	sub r2, r6   # Decrement counter
	ldi r6, 8    # Doesn't alter flags
	bf readloop  # Always branches, because r2 > 0
lastread:
	stf r2, 0x18 # Copy r2 DWs (r2 is always > 0)	

clearevent:
	stf r4, 0x14 # To MDA, address is frozen
	stf r4, 0x2b # Write a word (data ignored) and flush

	bf start     # Done. No interrupt anyhow.
	bt start     # In case the previous one didn't catch

write:
	ldf r3, 0x2b # r3 = Unaligned bytes to write
	stf r1, 0x04 # To MDA, address is incremental
			
	mov r0, r7   # Get a copy of the status register
	andi r0, 3   # r0 = number of byte writes

	loop endloop, 0
	stf r3, 0x29 # Write r3's LSB byte and flush
	rorb r3      # Rotate one byte (kinda little endian)		
endloop:
	cmpeqi r2, 0 # Is r2 = 0?
	bt checkint  # If so, we're done

writeloop:	
	stf r5, 0x00 # To MSA, NO prefetch, address is incremental
	cmphs r6, r2 # Is r6 larger or equal to the number of DWs left to copy?
	bt lastwrite # If so, jump to last transfer label
	stf r6, 0x18 # Copy r6 words from MSA to MDA address.
	yield  
	sub r2, r6   # Decrement counter
	ldi r6, 8    # Doesn't alter flags
	bf writeloop # Always branches, because r2 > 0
lastwrite:
	stf r2, 0x18 # Copy r2 DWs (r2 is always > 0)

checkint:
	btsti r7, 3    # Check bit 3 (Interrupt after write)
	bf clearevent  # If zero, no interrupt

	done 3         # Induce interrupt
	bt clearevent  # Always branches

	
