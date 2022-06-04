.data
    n: .word 11
.text
.globl __start

FUNCTION:
  # Todo: Define your own function in HW1
  addi sp, sp, -16 # save return address and n on stack
  sw x1, 8(sp)
  sw a0, 0(sp)
  addi x6, x0, 10
  addi x14,x0, 1
  bge a0, x6, L2 # case : n >= 10
  bge a0, x14, L1 # case : n >= 1
  addi t0, x0, 7 # basecase
  addi sp, sp, 16
  jalr x0, 0(x1) # return
L1: # case : n >= 1
  addi a0, a0, -1 # n = n-1
  jal x1, FUNCTION # call T(n-1)
  lw a0, 0(sp)
  lw x1, 8(sp)
  addi sp, sp, 16
  slli t0, t0, 1 # 2*T(n-1)
  jalr x0, 0(x1) # return
L2: # case : n >= 10
  addi x7, x0, 3
  mul a0, a0, x7
  srli a0, a0, 2
  jal x1,FUNCTION # call T(floor(3n/4))
  lw a0, 0(sp)
  lw x1, 8(sp)
  addi sp, sp, 16
  slli t0, t0, 1 # 2*T(floor(3n/4))
  addi x28, x0, 7
  mul x29, a0, x28
  srli x29, x29, 3
  addi x29, x29, -137 # 0.875n - 137
  add t0, t0, x29
  jalr x0, 0(x1) # return

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   a0, n
    sw   t0, 4(a0)
    addi a0,x0,10
    ecall