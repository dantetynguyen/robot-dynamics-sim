// parser.js — turns equation text into a syntax tree. Nothing here executes
// anything: there is no eval and no `new Function` anywhere in this project.
// The tree is later walked by compile.js, which only knows a fixed whitelist
// of operations.
//
// The syntax is a small MATLAB-like subset:
//
//   numbers        2, 0.5, 1e-3
//   names          G, Vd, dtheta            (variables supplied by the app)
//   arithmetic     + - * /  ^               (matrix rules: A*B is a matrix product)
//   element-wise   .*  ./  .^
//   transpose      A'
//   calls          ad(V), Ad(T), skew(w), eye(3), zeros(3,1), …
//   indexing       V(1:3), M(2,3), M(:,1)   (1-based, like MATLAB)
//   matrices       [a, b; c, d]             (commas between entries, ; between rows)
//   comments       % to end of line
//
// Grammar (lowest to highest precedence):
//   expr    := term (('+' | '-') term)*
//   term    := unary (('*' | '/' | '.*' | './') unary)*
//   unary   := ('-' | '+') unary | power
//   power   := postfix (('^' | '.^') unary)?
//   postfix := primary ("'")*
//   primary := number | name | name '(' args ')' | '(' expr ')' | '[' rows ']'

export class EquationError extends Error {
  constructor(message, position = null) {
    super(message);
    this.name = 'EquationError';
    this.position = position;
  }
}

const MAX_LENGTH = 2000;   // characters; keeps pathological input cheap
const MAX_DEPTH = 60;      // nesting depth

function tokenize(text) {
  if (text.length > MAX_LENGTH) throw new EquationError(`expression is too long (limit ${MAX_LENGTH} characters)`);
  const tokens = [];
  let i = 0;
  while (i < text.length) {
    const ch = text[i];
    if (ch === ' ' || ch === '\t' || ch === '\n' || ch === '\r') { i++; continue; }
    if (ch === '%') { while (i < text.length && text[i] !== '\n') i++; continue; }
    if ((ch >= '0' && ch <= '9') || (ch === '.' && text[i + 1] >= '0' && text[i + 1] <= '9')) {
      const m = /^(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?/.exec(text.slice(i));
      tokens.push({ type: 'num', value: parseFloat(m[0]), pos: i });
      i += m[0].length;
      continue;
    }
    if (/[A-Za-z_]/.test(ch)) {
      const m = /^[A-Za-z_][A-Za-z0-9_]*/.exec(text.slice(i));
      tokens.push({ type: 'name', value: m[0], pos: i });
      i += m[0].length;
      continue;
    }
    const two = text.slice(i, i + 2);
    if (two === '.*' || two === './' || two === '.^') {
      tokens.push({ type: 'op', value: two, pos: i });
      i += 2;
      continue;
    }
    if ('+-*/^\'()[],;:'.includes(ch)) {
      tokens.push({ type: 'op', value: ch, pos: i });
      i++;
      continue;
    }
    throw new EquationError(`unexpected character "${ch}"`, i);
  }
  tokens.push({ type: 'end', value: 'end of expression', pos: text.length });
  return tokens;
}

export function parse(text) {
  const tokens = tokenize(text);
  let k = 0;
  let depth = 0;

  const peek = () => tokens[k];
  const isOp = (v) => tokens[k].type === 'op' && tokens[k].value === v;
  const take = () => tokens[k++];
  const expect = (v) => {
    if (!isOp(v)) throw new EquationError(`expected "${v}" but found "${peek().value}"`, peek().pos);
    return take();
  };
  const enter = () => { if (++depth > MAX_DEPTH) throw new EquationError('expression is nested too deeply'); };
  const leave = () => { depth--; };

  function expr() {
    enter();
    let node = term();
    while (isOp('+') || isOp('-')) {
      const op = take();
      node = { type: 'binary', op: op.value, a: node, b: term(), pos: op.pos };
    }
    leave();
    return node;
  }

  function term() {
    let node = unary();
    while (isOp('*') || isOp('/') || isOp('.*') || isOp('./')) {
      const op = take();
      node = { type: 'binary', op: op.value, a: node, b: unary(), pos: op.pos };
    }
    return node;
  }

  function unary() {
    if (isOp('-')) { const op = take(); enter(); const x = unary(); leave(); return { type: 'neg', x, pos: op.pos }; }
    if (isOp('+')) { take(); return unary(); }
    return power();
  }

  function power() {
    const base = postfix();
    if (isOp('^') || isOp('.^')) {
      const op = take();
      enter();
      const exponent = unary();
      leave();
      return { type: 'binary', op: op.value, a: base, b: exponent, pos: op.pos };
    }
    return base;
  }

  function postfix() {
    let node = primary();
    while (isOp("'")) { const op = take(); node = { type: 'transpose', x: node, pos: op.pos }; }
    return node;
  }

  // One argument of f(...) or X(...). Ranges (a:b) and a bare colon are only
  // meaningful when indexing; compile.js rejects them elsewhere.
  function argument() {
    if (isOp(':')) { const op = take(); return { type: 'colon', pos: op.pos }; }
    const a = expr();
    if (isOp(':')) { const op = take(); return { type: 'range', a, b: expr(), pos: op.pos }; }
    return a;
  }

  function primary() {
    const tok = take();
    if (tok.type === 'num') return { type: 'num', value: tok.value, pos: tok.pos };
    if (tok.type === 'name') {
      if (!isOp('(')) return { type: 'name', name: tok.value, pos: tok.pos };
      take();
      const args = [];
      if (!isOp(')')) {
        args.push(argument());
        while (isOp(',')) { take(); args.push(argument()); }
      }
      expect(')');
      return { type: 'apply', name: tok.value, args, pos: tok.pos };
    }
    if (tok.type === 'op' && tok.value === '(') {
      const node = expr();
      expect(')');
      return node;
    }
    if (tok.type === 'op' && tok.value === '[') {
      const rows = [[]];
      if (!isOp(']')) {
        rows[0].push(expr());
        for (;;) {
          if (isOp(',')) { take(); rows[rows.length - 1].push(expr()); }
          else if (isOp(';')) { take(); if (isOp(']')) break; rows.push([expr()]); }
          else break;
        }
      }
      expect(']');
      if (rows[0].length === 0) throw new EquationError('empty matrix [] is not allowed', tok.pos);
      return { type: 'matrix', rows, pos: tok.pos };
    }
    throw new EquationError(`unexpected "${tok.value}"`, tok.pos);
  }

  const tree = expr();
  if (peek().type !== 'end') throw new EquationError(`unexpected "${peek().value}"`, peek().pos);
  return tree;
}
