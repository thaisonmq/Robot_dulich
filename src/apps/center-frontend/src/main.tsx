import ReactDOM from "react-dom/client";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { App } from "./App";
import { useAppStore } from "./state/appStore";
import "./styles.css";

const savedUser = sessionStorage.getItem("rovera_user");
if (savedUser) {
  try {
    useAppStore.getState().setUser(JSON.parse(savedUser));
  } catch {
    sessionStorage.removeItem("rovera_user");
  }
}

const queryClient = new QueryClient({
  defaultOptions: {
    queries: { retry: 2, staleTime: 1500 },
    mutations: { retry: false },
  },
});

ReactDOM.createRoot(document.getElementById("root")!).render(
  <QueryClientProvider client={queryClient}>
    <App />
  </QueryClientProvider>,
);
