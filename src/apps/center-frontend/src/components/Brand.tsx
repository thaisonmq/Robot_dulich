export function Brand({ compact = false }: { compact?: boolean }) {
  return (
    <div className={`brand ${compact ? "brand--compact" : ""}`}>
      <img
        src="/assets/mq-ict-solutions-logo.png"
        alt="MQ ICT Solutions — Powered by AI"
      />
    </div>
  );
}
