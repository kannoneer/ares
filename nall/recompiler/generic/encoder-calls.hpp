#pragma once

//{
  struct imm64 {
    explicit imm64(u64 data) : data(data) {}
    template<typename T> explicit imm64(T* pointer) : data((u64)pointer) {}
    template<typename C, typename R, typename... P> explicit imm64(auto (C::*function)(P...) -> R) {
      union force_cast_ub {
        auto (C::*function)(P...) -> R;
        u64 pointer;
      } cast{function};
      data = cast.pointer;
    }
    template<typename C, typename R, typename... P> explicit imm64(auto (C::*function)(P...) const -> R) {
      union force_cast_ub {
        auto (C::*function)(P...) const -> R;
        u64 pointer;
      } cast{function};
      data = cast.pointer;
    }
    u64 data;
  };

  template<typename C, typename V, typename... P>
  alwaysinline auto call(V (C::*function)(P...)) {
    static_assert(sizeof...(P) <= 3);
    #if 0
    sljit_s32 type = SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 1);
    if constexpr(sizeof...(P) >= 1) type |= SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 2);
    if constexpr(sizeof...(P) >= 2) type |= SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 3);
    if constexpr(sizeof...(P) >= 3) type |= SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 4);
    sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R0, 0, SLJIT_S0, 0);
#else
    
    sljit_s32 type = 0;

    // 	void *instruction, sljit_u32 size);

    // sljit's convention       amd64 ABI on Linux
    // reg(0) = rax             arg0 = %rdi
    // reg(1) = rsi, #1 arg     arg1 = %rsi
    // reg(2) = rdi, #2 arg     arg2 = %rdx
    // reg(3) = rcx, #3 arg     arg3 = %rcx
    // 

    // baseline, emits the same code as master above
    // ---------------------------------------------

    // u8 mov_rax_rbx[] = {0x48, 0x89, 0xd8};
    // sljit_emit_op_custom(compiler, mov_rax_rbx, sizeof(mov_rax_rbx));
    // if constexpr(sizeof...(P) >= 1) {
    //   // rsi should be already set
    // }
    // if constexpr(sizeof...(P) >= 2) {
    //   u8 mov_rdx_rdi[] = {0x48,0x89,0xfa};
    //   sljit_emit_op_custom(compiler, mov_rdx_rdi, sizeof(mov_rdx_rdi));
    // }
    // if constexpr(sizeof...(P) >= 3) {
    //   // rcx should be already set
    // }
    // u8 mov_rdi_rax[] = {0x48, 0x89, 0xc7};
    // sljit_emit_op_custom(compiler, mov_rdi_rax, sizeof(mov_rdi_rax));

    // alternative, this crashes at runtime
    // ------------------------------------

    // trying to replace
    // mov rax, rbx
    // mov rdx, rdi
    // mov rbx, rax

    // with just

    // mov rdx, rdi
    // mov rdi, rbx

    if constexpr(sizeof...(P) >= 1) {
      // rsi should be already set
    }
    if constexpr(sizeof...(P) >= 2) {
      u8 mov_rdx_rdi[] = {0x48,0x89,0xfa};
      sljit_emit_op_custom(compiler, mov_rdx_rdi, sizeof(mov_rdx_rdi));
    }
    if constexpr(sizeof...(P) >= 3) {
      // rcx should be already set
    }

    // Always pass 'this' as argument 0 in rdi
    u8 mov_rdi_rbx[] = {0x48,0x89,0xd8};
    sljit_emit_op_custom(compiler, mov_rdi_rbx, sizeof(mov_rdi_rbx));
    #endif

    if constexpr(!std::is_void_v<V>) type |= SLJIT_ARG_RETURN(SLJIT_ARG_TYPE_W);

    sljit_emit_icall(compiler, SLJIT_CALL, type, SLJIT_IMM, SLJIT_FUNC_ADDR(imm64{function}.data));
  }

  template<typename C, typename R, typename... P>
  alwaysinline auto call(auto (C::*function)(P...) -> R, C* object) {
    sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R0, 0, SLJIT_IMM, imm64{object}.data);
    sljit_s32 type = SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 1);
    if constexpr(!std::is_void_v<R>) type |= SLJIT_ARG_RETURN(SLJIT_ARG_TYPE_W);
    sljit_emit_icall(compiler, SLJIT_CALL, type, SLJIT_IMM, SLJIT_FUNC_ADDR(imm64{function}.data));
  }

  template<typename C, typename R, typename... P, typename P0>
  alwaysinline auto call(auto (C::*function)(P...) -> R, C* object, P0 p0) {
    sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R0, 0, SLJIT_IMM, imm64{object}.data);
    sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R1, 0, SLJIT_IMM, imm64(p0).data);
    sljit_s32 type = SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 1)
                   | SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 2);
    if constexpr(!std::is_void_v<R>) type |= SLJIT_ARG_RETURN(SLJIT_ARG_TYPE_W);
    sljit_emit_icall(compiler, SLJIT_CALL, type, SLJIT_IMM, SLJIT_FUNC_ADDR(imm64{function}.data));
  }

  template<typename C, typename R, typename... P, typename P0, typename P1>
  alwaysinline auto call(auto (C::*function)(P...) -> R, C* object, P0 p0, P1 p1) {
    sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R0, 0, SLJIT_IMM, imm64{object}.data);
    sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R1, 0, SLJIT_IMM, imm64(p0).data);
    sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R2, 0, SLJIT_IMM, imm64(p1).data);
    sljit_s32 type = SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 1)
                   | SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 2)
                   | SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 3);
    if constexpr(!std::is_void_v<R>) type |= SLJIT_ARG_RETURN(SLJIT_ARG_TYPE_W);
    sljit_emit_icall(compiler, SLJIT_CALL, type, SLJIT_IMM, SLJIT_FUNC_ADDR(imm64{function}.data));
  }

  template<typename C, typename R, typename... P, typename P0, typename P1, typename P2>
  alwaysinline auto call(auto (C::*function)(P...) -> R, C* object, P0 p0, P1 p1, P2 p2) {
    sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R0, 0, SLJIT_IMM, imm64{object}.data);
    sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R1, 0, SLJIT_IMM, imm64(p0).data);
    sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R2, 0, SLJIT_IMM, imm64(p1).data);
    sljit_emit_op1(compiler, SLJIT_MOV, SLJIT_R3, 0, SLJIT_IMM, imm64(p2).data);
    sljit_s32 type = SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 1)
                   | SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 2)
                   | SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 3)
                   | SLJIT_ARG_VALUE(SLJIT_ARG_TYPE_W, 4);
    if constexpr(!std::is_void_v<R>) type |= SLJIT_ARG_RETURN(SLJIT_ARG_TYPE_W);
    sljit_emit_icall(compiler, SLJIT_CALL, type, SLJIT_IMM, SLJIT_FUNC_ADDR(imm64{function}.data));
  }
//};
