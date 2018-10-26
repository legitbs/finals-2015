.intel_syntax noprefix

.globl NEXT_STATE
.globl SAVE_EVENT_STATE
.globl SWAP_TO_STATE
.globl SWAP_TO_EVENT
.globl _NEXT_STATE
.globl _SAVE_EVENT_STATE
.globl _SWAP_TO_STATE
.globl _SWAP_TO_EVENT
.globl CREATE_GLOBAL_STATE
.globl _CREATE_GLOBAL_STATE
.globl RESTORE_GLOBAL_STATE
.globl _RESTORE_GLOBAL_STATE
.globl UPDATE_EVENT_STATE
.globl _UPDATE_EVENT_STATE
.globl calloc
.globl CurrentState

#define PARAM0			4
#define PARAM1			8
#define STATE_CPU_EAX		0
#define STATE_CPU_EBX		STATE_CPU_EAX+4
#define STATE_CPU_ECX		STATE_CPU_EBX+4
#define STATE_CPU_EDX		STATE_CPU_ECX+4
#define STATE_CPU_EDI		STATE_CPU_EDX+4
#define STATE_CPU_ESI		STATE_CPU_EDI+4
#define STATE_CPU_EBP		STATE_CPU_ESI+4
#define STATE_CPU_ESP		STATE_CPU_EBP+4
#define STATE_ORIGPC		STATE_CPU_ESP+4
#define STATE_CURSTATE		STATE_ORIGPC+4
#define STATE_EVENTLEVEL	STATE_CURSTATE+4
#define STATE_STACK		STATE_EVENTLEVEL+4
#define STATE_ORIGSTACK		STATE_STACK+4
#define STATE_FUNCTION		STATE_ORIGSTACK+4
#define STATE_TRIGGERCOUNT	STATE_FUNCTION+4
#define STATE_TRIGGERS		STATE_TRIGGERCOUNT+4
#define ROBOT_EVENTS		(8*10)

SAVE_EVENT_STATE:
_SAVE_EVENT_STATE:
	push ebx

	//get the param passed in, +4 due to push
	mov ebx, [esp+PARAM0+4]

	//store off all of the regs
	mov [ebx+STATE_CPU_EAX], eax
	pop eax
	mov [ebx+STATE_CPU_EBX], eax
	mov [ebx+STATE_CPU_ECX], ecx
	mov [ebx+STATE_CPU_EDX], edx
	mov [ebx+STATE_CPU_EDI], edi
	mov [ebx+STATE_CPU_ESI], esi

	mov [ebx+STATE_ORIGSTACK], esp		//backup of the original stack
	
	//store off our pointer to the event itself
	//allocate a new stack

    //stupid fucking llvm can't do a push properly in 32bit x86
    //push dword ptr 16384 -> pushw 0x2000
    //pushl 16384 -> invalid instruction
    //push 16384 -> pushw 0x2000
    //push 0x00002000 -> pushw 0x2000
    //push qword ptr 16384 -> pushw 0x2000
    //just encode the stupid push
    .byte 0x68, 0x00, 0x40, 0x00, 0x00
    .byte 0x68, 0x01, 0x00, 0x00, 0x00
	call calloc

	pop ecx
	pop ecx
    
	//assign to the stack entry in the state structure
	mov [ebx+STATE_STACK], eax

    //calculate the stack frame size and update ebp
	mov ecx, ebp
	sub ecx, esp

	//copy data to the new stack
	lea edi, [eax+16384]

    //save off new ebp
    mov [ebx+STATE_CPU_EBP], edi
    
    	sub edi, ecx
	mov esi, esp

	//assign the new stack to the stored information
	mov [ebx+STATE_CPU_ESP], edi

	rep movsb

    //if the param is within our stack then we need to adjust ebx to the new stack
    //if we are swapping to it
    cmp dword ptr [esp+PARAM1], 1
    jnz SkipUpdateCheck

    //we are going to be swapping, see if our incoming area is within
    //the stack we are copying, if so then adjust ebx to match our new stack
    //and not the old one
    cmp ebx, esp
    jb SkipSaveCheck
    cmp ebx, ebp
    jg SkipSaveCheck

    //ebx must be within esp/ebp range
    //update accordingly
    mov ecx, ebp
    sub ecx, ebx

    mov ebx, [ebx+STATE_CPU_EBP]
    sub ebx, ecx

SkipSaveCheck:
    //save off the original PC
    mov eax, [esp]
    mov [ebx+STATE_ORIGPC], eax

    //if this is a request to save and swap then do so
    cmp dword ptr [esp+PARAM1], 1
    je Swap_to_State_Continue

    //restore the registers we modified
    mov eax, [ebx+STATE_CPU_EAX]
    mov ecx, [ebx+STATE_CPU_ECX]
    mov edx, [ebx+STATE_CPU_EDX]
    mov edi, [ebx+STATE_CPU_EDI]
    mov esi, [ebx+STATE_CPU_ESI]
    mov ebx, [ebx+STATE_CPU_EBX]
    ret

UPDATE_EVENT_STATE:
_UPDATE_EVENT_STATE:
    //do everything except create a new stack
    //just re-use the created stack
	push ebx

    //get the param passed in
    mov ebx, [esp+PARAM0+4]

	//store off all of the regs
	mov [ebx+STATE_CPU_EAX], eax
    pop eax
	mov [ebx+STATE_CPU_EBX], eax
	mov [ebx+STATE_CPU_ECX], ecx
	mov [ebx+STATE_CPU_EDX], edx
	mov [ebx+STATE_CPU_EDI], edi
	mov [ebx+STATE_CPU_ESI], esi

	mov [ebx+STATE_ORIGSTACK], esp		//backup of the original stack

    //calculate the stack frame size and update ebp
    mov ecx, ebp
    sub ecx, esp

	//copy data to the stack
    mov edi, [ebx+STATE_STACK]
    add edi, 16384

    //save off new ebp
    mov [ebx+STATE_CPU_EBP], edi

    sub edi, ecx
	mov esi, esp

	//assign the new stack to the stored information
	mov [ebx+STATE_CPU_ESP], edi

	rep movsb

    //if the param is within our stack then we need to adjust ebx to the new stack
    //if we are swapping to it
    cmp dword ptr [esp+PARAM1], 1
    jnz SkipUpdateCheck

    //we are going to be swapping, see if our incoming area is within
    //the stack we are copying, if so then adjust ebx to match our new stack
    //and not the old one
    cmp ebx, esp
    jb SkipUpdateCheck
    cmp ebx, ebp
    jg SkipUpdateCheck

    //ebx must be within esp/ebp range
    //update accordingly
    mov ecx, ebp
    sub ecx, ebx

    mov ebx, [ebx+STATE_CPU_EBP]
    sub ebx, ecx

SkipUpdateCheck:
    //save off the original PC
    mov eax, [esp]
    mov [ebx+STATE_ORIGPC], eax

    //if this is a request to save and swap then do so
    cmp dword ptr [esp+PARAM1], 1
    je Swap_to_State_Continue

    //restore the registers we modified
    mov eax, [ebx+STATE_CPU_EAX]
    mov ecx, [ebx+STATE_CPU_ECX]
    mov edx, [ebx+STATE_CPU_EDX]
    mov edi, [ebx+STATE_CPU_EDI]
    mov esi, [ebx+STATE_CPU_ESI]
    mov ebx, [ebx+STATE_CPU_EBX]
    ret

CREATE_GLOBAL_STATE:
_CREATE_GLOBAL_STATE:
	push ebx

    //get the param passed in
    mov ebx, [esp+PARAM0+4]

	//store off all of the regs
	mov [ebx+STATE_CPU_EAX], eax
	pop eax
	mov [ebx+STATE_CPU_EBX], eax
	mov [ebx+STATE_CPU_ECX], ecx
	mov [ebx+STATE_CPU_EDX], edx
	mov [ebx+STATE_CPU_EDI], edi
	mov [ebx+STATE_CPU_ESI], esi
	mov [ebx+STATE_CPU_EBP], ebp
	mov [ebx+STATE_CPU_ESP], esp
	mov [ebx+STATE_ORIGSTACK], esp		//backup of the original stack

	//assign to the stack entry in the state structure
	mov [ebx+STATE_STACK], esp

    //save off the original PC
    mov eax, [esp]
    mov [ebx+STATE_ORIGPC], eax

    //restore ebx
    mov ebx, [ebx+STATE_CPU_EBX]
    ret

NEXT_STATE:
_NEXT_STATE:
	mov ebx, [CurrentState]
	
	//save off the new state that is passed in
    mov eax, [esp+PARAM0]
	mov [ebx+STATE_CURSTATE], eax
	
	//restore all registers
        mov ecx, [ebx+STATE_CPU_ECX]
        mov edx, [ebx+STATE_CPU_EDX]
        mov edi, [ebx+STATE_CPU_EDI]
        mov esi, [ebx+STATE_CPU_ESI]
        mov ebp, [ebx+STATE_CPU_EBP]
        mov esp, [ebx+STATE_CPU_ESP]

    //restore original PC just incase
    mov eax, [ebx+STATE_ORIGPC]
    mov [esp], eax

    //continue restoring registers
	mov eax, [ebx+STATE_CPU_EAX]
	mov ebx, [ebx+STATE_CPU_EBX]
	ret
	
SWAP_TO_STATE:
_SWAP_TO_STATE:
	//save off the current state we are working on
    mov ebx, [esp+PARAM0]
Swap_to_State_Continue:
	mov [CurrentState], ebx

	//restore all registers
        mov ecx, [ebx+STATE_CPU_ECX]
        mov edx, [ebx+STATE_CPU_EDX]
        mov edi, [ebx+STATE_CPU_EDI]
        mov esi, [ebx+STATE_CPU_ESI]
        mov ebp, [ebx+STATE_CPU_EBP]
        mov esp, [ebx+STATE_CPU_ESP]

    //restore original PC just incase
    mov eax, [ebx+STATE_ORIGPC]
    mov [esp], eax

    //continue restoring registers
	mov eax, [ebx+STATE_CPU_EAX]
	mov ebx, [ebx+STATE_CPU_EBX]
	ret

RESTORE_GLOBAL_STATE:
_RESTORE_GLOBAL_STATE:
	//save off the current state we are working on
    mov ebx, [esp+PARAM0]
	mov [CurrentState], ebx

	//restore all registers then return to where we were called from
    //unlike SWAP_TO_STATE that restores the PC too

        mov eax, [ebx+STATE_CPU_EAX]
        mov ebx, [ebx+STATE_CPU_EBX]
        mov ecx, [ebx+STATE_CPU_ECX]
        mov edx, [ebx+STATE_CPU_EDX]
        mov edi, [ebx+STATE_CPU_EDI]
        mov esi, [ebx+STATE_CPU_ESI]
        mov ebp, [ebx+STATE_CPU_EBP]
        mov esp, [ebx+STATE_CPU_ESP]
	ret

SWAP_TO_EVENT:
_SWAP_TO_EVENT:
	//save off the current state we are working on
    mov ebx, [esp+PARAM0]
    mov eax, [esp+PARAM1]
	mov ebx, [ebx+(eax*4)+ROBOT_EVENTS]
	mov [CurrentState], ebx
	
	//restore all registers
        mov ecx, [ebx+STATE_CPU_ECX]
        mov edx, [ebx+STATE_CPU_EDX]
        mov edi, [ebx+STATE_CPU_EDI]
        mov esi, [ebx+STATE_CPU_ESI]
        mov ebp, [ebx+STATE_CPU_EBP]
        mov esp, [ebx+STATE_CPU_ESP]

    //restore original PC just incase
    mov eax, [ebx+STATE_ORIGPC]
    mov [esp], eax

    //continue restoring registers
	mov eax, [ebx+STATE_CPU_EAX]
	mov ebx, [ebx+STATE_CPU_EBX]
	ret
